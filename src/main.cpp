/*
* Vulkan physical based rendering glTF 2.0 demo
*
* Copyright (C) 2018 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

// glTF format: https://github.com/KhronosGroup/glTF
// tinyglTF loader: https://github.com/syoyo/tinygltf

/* 
 * Modifications by Håkon Flatval, February 2020:
 * - Add support for taking screenshots
 */

#define OUTPUT_INDEX_PAD 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <vector>
#include <chrono>
#include <map>
#include "algorithm"

#include "unistd.h"

#if defined(__ANDROID__)
#define TINYGLTF_ANDROID_LOAD_FROM_ASSETS
#endif

#include <vulkan/vulkan.h>

#define CUSTOM_FORMAT VK_FORMAT_R32G32B32A32_SFLOAT
// #define CUSTOM_FORMAT VK_FORMAT_R8G8B8A8_UNORM

#include "VulkanExampleBase.h"
#include "VulkanTexture.hpp"
#include "VulkanglTFModel.hpp"
#include "VulkanUtils.hpp"

#ifdef WITH_DISPLAY
#include "ui.hpp"
#endif // WITH_DISPLAY

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <OpenImageIO/imageio.h>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

/*
  Util function(s)
*/

void convert_to_uint8(float* data, uint8_t* out, int width, int height) {
    // Assume float data contains components within [0.0, 1.0]
    for(int i = 0; i < width * height * 4; i++) {
	out[i] = (uint8_t)(data[i] * 255);
    }
}

// Normalize float values to the range [0, 255], separately for each channel
void normalize_image_buffer(float* data, uint8_t* out, int width, int height) {
  float biggest[4] = {-1e6, -1e6, -1e6, -1e6};
    float smallest[4] = {1e6, 1e6, 1e6, 1e6};
    for(int i = 0; i < width * height; i++) {
	for(int j = 0; j < 4; j++) {
	    biggest[j] = std::max(biggest[j], data[4 * i + j]); // (unsigned char)((data[i] >> (8 * j)) & 255));
	    smallest[j] = std::min(smallest[j], data[4 * i + j]);
	}
    }

    float invdiffs[4];
    
    for(int i = 0; i < 4; i++) {
	std::cout << "Smallest: " << smallest[i] << "\nBiggest: " << biggest[i] << std::endl;
	invdiffs[i] = 1.0f / (biggest[i] - smallest[i]);
    }


    for(int i = 0; i < width * height; i++) {
	/* int ll = 0;
	for(int j = 0 ; j < 3; j++) {
	    ll |= (((data[i] >> (8 * j)) & 255) * 255 / biggest[j]) << (8 * j);
	}
	data[i] = ll | (255 << 24); */
	for(int j = 0; j < 3; j++) {
	    out[4 * i + j] = (uint8_t)((data[4 * i + j] - smallest[j]) * invdiffs[j] * 255);
	}
	out[4 * i + 3] = 255;
    }
}

// Convert from four channels to three
void to3chan(float* data, int width, int height) {
  for(int i = 0; i < width * height; i++) {
    data[3 * i + 0] = data[4 * i + 0];
    data[3 * i + 1] = data[4 * i + 1];
    data[3 * i + 2] = data[4 * i + 2];
  }
}

void output_image_float(float* data, int width, int height, int channels, const std::string& file_name) {

  if(channels != 3) {
    std::cerr << "Number of channels must be 3 for the time being (for input to BMFR)" << std::endl;
    exit(-1);
  }
  /* for(int i = 0; i < width * height * 4; i++ ) {
    data[i] = 3.0f;
    } */
  std::unique_ptr<OIIO::ImageOutput> out = OIIO::ImageOutput::create(file_name);

  if(!out) {
    std::cerr << "Cannot open output path " << file_name << ", quitting" << std::endl;
    exit(-1);
  }
  
  OIIO::ImageSpec spec(width, height, 3, OIIO::TypeDesc::FLOAT);
  out->open(file_name, spec);
  out->write_image(OIIO::TypeDesc::FLOAT, data + 3 * width * (height - 1),
		   OIIO::AutoStride,
		   - width * 3 * sizeof(float)); // Output image upside-down
  out->close();
}

/* void output_image_uint8(float* data, int width, int height, const std::string& file_name) {
  
   } */

/*
	PBR example main class
*/
class VulkanExample : public VulkanExampleBase
{
public:
	struct Textures {
		vks::TextureCubeMap environmentCube;
		vks::Texture2D empty;
		vks::Texture2D lutBrdf;
		vks::TextureCubeMap irradianceCube;
		vks::TextureCubeMap prefilteredCube;
	} textures;

	struct Models {
		vkglTF::Model scene;
		vkglTF::Model skybox;
	} models;

	struct UniformBufferSet {
		Buffer scene;
		Buffer skybox;
		Buffer params;
	};

	struct UBOMatrices {
		glm::mat4 projection;
		glm::mat4 model;
		glm::mat4 view;
		glm::vec3 camPos;
	} shaderValuesScene, shaderValuesSkybox;

	struct shaderValuesParams {
		glm::vec4 lightDir;
		float exposure = 4.5f;
		float gamma = 2.2f;
		float prefilteredCubeMipLevels;
		float scaleIBLAmbient = 1.0f;
		float debugViewInputs = 0;
		float debugViewEquation = 0;
	} shaderValuesParams;

	VkPipelineLayout pipelineLayout;

	struct Pipelines {
		VkPipeline skybox;
		VkPipeline pbr;
		VkPipeline pbrAlphaBlend;
	} pipelines;

	struct DescriptorSetLayouts {
		VkDescriptorSetLayout scene;
		VkDescriptorSetLayout material;
		VkDescriptorSetLayout node;
	} descriptorSetLayouts;

	struct DescriptorSets {
		VkDescriptorSet scene;
		VkDescriptorSet skybox;
	};


    struct CustomStuff {
	struct {
	    VkImage image;
	    VkImageView view;
	    VkDeviceMemory memory;
	} fbColor;

        struct {
	    VkImage image;
	    VkDeviceMemory memory;
	    VkDeviceSize memorySize;
	} reachableImage;

	struct {
	    VkImage image;
	    VkImageView view;
	    VkDeviceMemory memory;
	} fbDepth;
	
	VkFramebuffer framebuffer;
	VkSemaphore renderedSemaphore;
	VkSemaphore copiedSemaphore;
	VkFence fence;
	
	VkCommandBuffer commandBuffers[4]; // Assume we don't have more than 4 swapchain buffers
	VkCommandBuffer secondCommandBuffer;
	VkRenderPass renderPass;
    } customStuff;
    
	std::vector<DescriptorSets> descriptorSets;

	std::vector<VkCommandBuffer> commandBuffers;
	std::vector<UniformBufferSet> uniformBuffers;

	std::vector<VkFence> waitFences;
	std::vector<VkSemaphore> renderCompleteSemaphores;
	std::vector<VkSemaphore> presentCompleteSemaphores;

	const uint32_t renderAhead = 2;
	uint32_t frameIndex = 0;

	int32_t animationIndex = 0;
	float animationTimer = 0.0f;
	bool animate = true;

	bool displayBackground = true;
	
	struct LightSource {
		glm::vec3 color = glm::vec3(1.0f);
		glm::vec3 rotation = glm::vec3(75.0f, 40.0f, 0.0f);
	} lightSource;

#ifdef WITH_DISPLAY
	UI *ui;
#endif // WITH_DISPLAY

#if defined(VK_USE_PLATFORM_ANDROID_KHR)
	const std::string assetpath = "";
#else
	const std::string assetpath = "./../data/";
#endif

	bool rotateModel = false;
	glm::vec3 modelrot = glm::vec3(0.0f);
	glm::vec3 modelPos = glm::vec3(0.0f);

	enum PBRWorkflows{ PBR_WORKFLOW_METALLIC_ROUGHNESS = 0, PBR_WORKFLOW_SPECULAR_GLOSINESS = 1 };

	struct PushConstBlockMaterial {
		glm::vec4 baseColorFactor;
		glm::vec4 emissiveFactor;
		glm::vec4 diffuseFactor;
		glm::vec4 specularFactor;
		float workflow;
		int colorTextureSet;
		int PhysicalDescriptorTextureSet;
		int normalTextureSet;
		int occlusionTextureSet;
		int emissiveTextureSet;
		float metallicFactor;
		float roughnessFactor;
		float alphaMask;
		float alphaMaskCutoff;
	} pushConstBlockMaterial;

	std::map<std::string, std::string> environments;
	std::string selectedEnvironment = "papermill";

#if !defined(_WIN32)
	std::map<std::string, std::string> scenes;
	std::string selectedScene = "DamagedHelmet";
#endif

	int32_t debugViewInputs = 0;
	int32_t debugViewEquation = 0;

	VulkanExample() : VulkanExampleBase()
	{
		title = "Vulkan glTF 2.0 PBR - � Sascha Willems (www.saschawillems.de)";
#if defined(TINYGLTF_ENABLE_DRACO)
		std::cout << "Draco mesh compression is enabled" << std::endl;
#endif
	}

	~VulkanExample()
	{
	    
	    destroyCustomStuff();
		vkDestroyPipeline(device, pipelines.skybox, nullptr);
		vkDestroyPipeline(device, pipelines.pbr, nullptr);
		vkDestroyPipeline(device, pipelines.pbrAlphaBlend, nullptr);

		vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
		vkDestroyDescriptorSetLayout(device, descriptorSetLayouts.scene, nullptr);
		vkDestroyDescriptorSetLayout(device, descriptorSetLayouts.material, nullptr);
		vkDestroyDescriptorSetLayout(device, descriptorSetLayouts.node, nullptr);

		models.scene.destroy(device);
		models.skybox.destroy(device);

		for (auto buffer : uniformBuffers) {
			buffer.params.destroy();
			buffer.scene.destroy();
			buffer.skybox.destroy();
		}
		for (auto fence : waitFences) {
			vkDestroyFence(device, fence, nullptr);
		}
		for (auto semaphore : renderCompleteSemaphores) {
			vkDestroySemaphore(device, semaphore, nullptr);
		}
		for (auto semaphore : presentCompleteSemaphores) {
			vkDestroySemaphore(device, semaphore, nullptr);
		}

		textures.environmentCube.destroy();
		textures.irradianceCube.destroy();
		textures.prefilteredCube.destroy();
		textures.lutBrdf.destroy();
		textures.empty.destroy();


#ifdef WITH_DISPLAY
		delete ui;
#endif // WITH_DISPLAY
	}

	void renderNode(vkglTF::Node *node, int32_t cbIndex, vkglTF::Material::AlphaMode alphaMode) {
	    
		if (node->mesh) {
			// Render mesh primitives
			for (vkglTF::Primitive * primitive : node->mesh->primitives) {
				if (primitive->material.alphaMode == alphaMode) {

					const std::vector<VkDescriptorSet> descriptorsets = {
											     descriptorSets[cbIndex <= -1 ? - cbIndex - 1 : cbIndex].scene,
						primitive->material.descriptorSet,
						node->mesh->uniformBuffer.descriptorSet,
					};
					vkCmdBindDescriptorSets(cbIndex <= -1 ? customStuff.commandBuffers[- cbIndex - 1]: commandBuffers[cbIndex], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, static_cast<uint32_t>(descriptorsets.size()), descriptorsets.data(), 0, NULL);

					// Pass material parameters as push constants
					PushConstBlockMaterial pushConstBlockMaterial{};					
					pushConstBlockMaterial.emissiveFactor = primitive->material.emissiveFactor;
					// To save push constant space, availabilty and texture coordiante set are combined
					// -1 = texture not used for this material, >= 0 texture used and index of texture coordinate set
					pushConstBlockMaterial.colorTextureSet = primitive->material.baseColorTexture != nullptr ? primitive->material.texCoordSets.baseColor : -1;
					pushConstBlockMaterial.normalTextureSet = primitive->material.normalTexture != nullptr ? primitive->material.texCoordSets.normal : -1;
					pushConstBlockMaterial.occlusionTextureSet = primitive->material.occlusionTexture != nullptr ? primitive->material.texCoordSets.occlusion : -1;
					pushConstBlockMaterial.emissiveTextureSet = primitive->material.emissiveTexture != nullptr ? primitive->material.texCoordSets.emissive : -1;
					pushConstBlockMaterial.alphaMask = static_cast<float>(primitive->material.alphaMode == vkglTF::Material::ALPHAMODE_MASK);
					pushConstBlockMaterial.alphaMaskCutoff = primitive->material.alphaCutoff;

					// TODO: glTF specs states that metallic roughness should be preferred, even if specular glosiness is present

					if (primitive->material.pbrWorkflows.metallicRoughness) {
						// Metallic roughness workflow
						pushConstBlockMaterial.workflow = static_cast<float>(PBR_WORKFLOW_METALLIC_ROUGHNESS);
						pushConstBlockMaterial.baseColorFactor = primitive->material.baseColorFactor;
						pushConstBlockMaterial.metallicFactor = primitive->material.metallicFactor;
						pushConstBlockMaterial.roughnessFactor = primitive->material.roughnessFactor;
						pushConstBlockMaterial.PhysicalDescriptorTextureSet = primitive->material.metallicRoughnessTexture != nullptr ? primitive->material.texCoordSets.metallicRoughness : -1;
						pushConstBlockMaterial.colorTextureSet = primitive->material.baseColorTexture != nullptr ? primitive->material.texCoordSets.baseColor : -1;
					}

					if (primitive->material.pbrWorkflows.specularGlossiness) {
						// Specular glossiness workflow
						pushConstBlockMaterial.workflow = static_cast<float>(PBR_WORKFLOW_SPECULAR_GLOSINESS);
						pushConstBlockMaterial.PhysicalDescriptorTextureSet = primitive->material.extension.specularGlossinessTexture != nullptr ? primitive->material.texCoordSets.specularGlossiness : -1;
						pushConstBlockMaterial.colorTextureSet = primitive->material.extension.diffuseTexture != nullptr ? primitive->material.texCoordSets.baseColor : -1;
						pushConstBlockMaterial.diffuseFactor = primitive->material.extension.diffuseFactor;
						pushConstBlockMaterial.specularFactor = glm::vec4(primitive->material.extension.specularFactor, 1.0f);
					}

					vkCmdPushConstants(cbIndex <= -1 ? customStuff.commandBuffers[- cbIndex - 1]: commandBuffers[cbIndex], pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushConstBlockMaterial), &pushConstBlockMaterial);

					if (primitive->hasIndices) {
						vkCmdDrawIndexed(cbIndex <= -1 ? customStuff.commandBuffers[- cbIndex - 1]: commandBuffers[cbIndex], primitive->indexCount, 1, primitive->firstIndex, 0, 0);
					} else {
						vkCmdDraw(cbIndex <= -1 ? customStuff.commandBuffers[- cbIndex - 1]: commandBuffers[cbIndex], primitive->vertexCount, 1, 0, 0);
					}
				}
			}

		};
		for (auto child : node->children) {
			renderNode(child, cbIndex, alphaMode);
		}
	}

    void recordCustomCommandBuffer(int ccb) {
	VkCommandBufferBeginInfo cmdBufferBeginInfo{};
	cmdBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	VkClearValue clearValues[2];
	clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 1.0f } };
	clearValues[1].depthStencil = { 1.0f, 0};

	VkRenderPassBeginInfo rpbi {};
	rpbi.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
	rpbi.renderPass = customStuff.renderPass;
	rpbi.renderArea.offset.x = 0;
	rpbi.renderArea.offset.y = 0;
	rpbi.renderArea.extent.width = this->width;
	rpbi.renderArea.extent.height = this->height;
	rpbi.clearValueCount = 2;
	rpbi.pClearValues = clearValues;
	rpbi.framebuffer = customStuff.framebuffer;

	VkCommandBuffer cb = customStuff.commandBuffers[ccb];

	VK_CHECK_RESULT(vkBeginCommandBuffer(cb, &cmdBufferBeginInfo));
	vkCmdBeginRenderPass(cb, &rpbi, VK_SUBPASS_CONTENTS_INLINE);

	VkViewport viewport{};
	viewport.width = (float)this->width;
	viewport.height = (float)this->height;
	viewport.minDepth = 0.0f;
	viewport.maxDepth = 1.0f;
	vkCmdSetViewport(cb, 0, 1, &viewport);

	VkRect2D scissor{};
	scissor.extent = { this->width, this->height };
	vkCmdSetScissor(cb, 0, 1, &scissor);

	VkDeviceSize offsets[1] = { 0 };

	if (displayBackground) {
	    vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets[ccb].skybox, 0, nullptr);
	    vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.skybox);
	    models.skybox.draw(cb);
	}

	vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.pbr);

	vkglTF::Model &model = models.scene;

	vkCmdBindVertexBuffers(cb, 0, 1, &model.vertices.buffer, offsets);
	if (model.indices.buffer != VK_NULL_HANDLE) {
	    vkCmdBindIndexBuffer(cb, model.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
	}

	for (auto node : model.nodes) {
	    renderNode(node, - ccb - 1, vkglTF::Material::ALPHAMODE_OPAQUE);
	}
	for (auto node : model.nodes) {
	    renderNode(node, - ccb - 1, vkglTF::Material::ALPHAMODE_MASK);
	}

	vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.pbrAlphaBlend);
	for (auto node : model.nodes) {
	    renderNode(node, - ccb - 1, vkglTF::Material::ALPHAMODE_BLEND);
	}

	vkCmdEndRenderPass(cb);
	VK_CHECK_RESULT(vkEndCommandBuffer(cb));
    }

	void recordCommandBuffers()
	{
	    std::cout << "Not recording normal command buffers, only offscreen ones" << std::endl;

	    for(size_t i = 0; i < commandBuffers.size(); i++) {
		
			recordCustomCommandBuffer(i);
	    } 
	    return;
		VkCommandBufferBeginInfo cmdBufferBeginInfo{};
		cmdBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

		VkClearValue clearValues[3];
		if (settings.multiSampling) {
			clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 1.0f } };
			clearValues[1].color = { { 0.0f, 0.0f, 0.0f, 1.0f } };
			clearValues[2].depthStencil = { 1.0f, 0 };
		}
		else {
			clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 1.0f } };
			clearValues[1].depthStencil = { 1.0f, 0 };
		}

		VkRenderPassBeginInfo renderPassBeginInfo{};
		renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
		renderPassBeginInfo.renderPass = renderPass;
		renderPassBeginInfo.renderArea.offset.x = 0;
		renderPassBeginInfo.renderArea.offset.y = 0;
		renderPassBeginInfo.renderArea.extent.width = width;
		renderPassBeginInfo.renderArea.extent.height = height;
		renderPassBeginInfo.clearValueCount = settings.multiSampling ? 3 : 2;
		renderPassBeginInfo.pClearValues = clearValues;

		for (size_t i = 0; i < commandBuffers.size(); ++i) {
			renderPassBeginInfo.framebuffer = frameBuffers[i];

			VkCommandBuffer currentCB = commandBuffers[i];

			VK_CHECK_RESULT(vkBeginCommandBuffer(currentCB, &cmdBufferBeginInfo));
			
			vkCmdBeginRenderPass(currentCB, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
			

			VkViewport viewport{};
			viewport.width = (float)width;
			viewport.height = (float)height;
			viewport.minDepth = 0.0f;
			viewport.maxDepth = 1.0f;
			vkCmdSetViewport(currentCB, 0, 1, &viewport);

			VkRect2D scissor{};
			scissor.extent = { width, height };
			vkCmdSetScissor(currentCB, 0, 1, &scissor);

			VkDeviceSize offsets[1] = { 0 };

			if (displayBackground) {
				vkCmdBindDescriptorSets(currentCB, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets[i].skybox, 0, nullptr);
				vkCmdBindPipeline(currentCB, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.skybox);
				// models.skybox.draw(currentCB);
			}

			vkCmdBindPipeline(currentCB, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.pbr);

			vkglTF::Model &model = models.scene;

			vkCmdBindVertexBuffers(currentCB, 0, 1, &model.vertices.buffer, offsets);
			if (model.indices.buffer != VK_NULL_HANDLE) {
				vkCmdBindIndexBuffer(currentCB, model.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
			}
			// Opaque primitives first
			for (auto node : model.nodes) {
				renderNode(node, i, vkglTF::Material::ALPHAMODE_OPAQUE);
			}

			// Alpha masked primitives
			for (auto node : model.nodes) {
				renderNode(node, i, vkglTF::Material::ALPHAMODE_MASK);
			}
			// Transparent primitives
			// TODO: Correct depth sorting
			vkCmdBindPipeline(currentCB, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.pbrAlphaBlend);
			for (auto node : model.nodes) {
				renderNode(node, i, vkglTF::Material::ALPHAMODE_BLEND);
			}

			// User interface
			// ui->draw(currentCB);

			vkCmdEndRenderPass(currentCB);
			VK_CHECK_RESULT(vkEndCommandBuffer(currentCB));
					
		}

	}

	void loadScene(std::string filename)
	{
		std::cout << "Loading scene from " << filename << std::endl;
		models.scene.destroy(device);
		animationIndex = 0;
		animationTimer = 0.0f;
		models.scene.loadFromFile(filename, vulkanDevice, queue);
		camera.setPosition({ 0.0f, 0.0f, 1.0f });
		camera.setRotation({ 0.0f, 0.0f, 0.0f });
	}

	void loadEnvironment(std::string filename)
	{
		std::cout << "Loading environment from " << filename << std::endl;
		if (textures.environmentCube.image) {
			textures.environmentCube.destroy();
			textures.irradianceCube.destroy();
			textures.prefilteredCube.destroy();
		}
		textures.environmentCube.loadFromFile(filename, VK_FORMAT_R16G16B16A16_SFLOAT, vulkanDevice, queue);
		generateCubemaps();
	}

	void loadAssets()
	{
#if defined(VK_USE_PLATFORM_ANDROID_KHR)
		tinygltf::asset_manager = androidApp->activity->assetManager;
		readDirectory(assetpath + "models", "*.gltf", scenes, true);
#else
		const std::string assetpath = "./../data/";
		struct stat info;
		if (stat(assetpath.c_str(), &info) != 0) {
			std::string msg = "Could not locate asset path in \"" + assetpath + "\".\nMake sure binary is run from correct relative directory!";
			std::cerr << msg << std::endl;
			exit(-1);
		}
#endif
		readDirectory(assetpath + "environments", "*.ktx", environments, false);

		textures.empty.loadFromFile(assetpath + "textures/empty.ktx", VK_FORMAT_R8G8B8A8_UNORM, vulkanDevice, queue);

		std::string sceneFile;
		if(settings.sceneFile.length()) {
		    sceneFile = settings.sceneFile;
		} else {
		    sceneFile = assetpath + "models/DamagedHelmet/glTF-Embedded/DamagedHelmet.gltf";
		}
		
		std::string envMapFile = assetpath + "environments/papermill.ktx";
		for (size_t i = 0; i < args.size(); i++) {
			if (std::string(args[i]).find(".gltf") != std::string::npos) {
				std::ifstream file(args[i]);
				if (file.good()) {
					sceneFile = args[i];
				} else {
					std::cout << "could not load \"" << args[i] << "\"" << std::endl;
				}
			}
			if (std::string(args[i]).find(".ktx") != std::string::npos) {
				std::ifstream file(args[i]);
				if (file.good()) {
					envMapFile = args[i];
				}
				else {
					std::cout << "could not load \"" << args[i] << "\"" << std::endl;
				}
			}
		}

		loadScene(sceneFile.c_str());
		models.skybox.loadFromFile(assetpath + "models/Box/glTF-Embedded/Box.gltf", vulkanDevice, queue);

		loadEnvironment(envMapFile.c_str());
	}

	void setupNodeDescriptorSet(vkglTF::Node *node) {
		if (node->mesh) {
			VkDescriptorSetAllocateInfo descriptorSetAllocInfo{};
			descriptorSetAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
			descriptorSetAllocInfo.descriptorPool = descriptorPool;
			descriptorSetAllocInfo.pSetLayouts = &descriptorSetLayouts.node;
			descriptorSetAllocInfo.descriptorSetCount = 1;
			VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &descriptorSetAllocInfo, &node->mesh->uniformBuffer.descriptorSet));

			VkWriteDescriptorSet writeDescriptorSet{};
			writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
			writeDescriptorSet.descriptorCount = 1;
			writeDescriptorSet.dstSet = node->mesh->uniformBuffer.descriptorSet;
			writeDescriptorSet.dstBinding = 0;
			writeDescriptorSet.pBufferInfo = &node->mesh->uniformBuffer.descriptor;

			vkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, nullptr);
		}
		for (auto& child : node->children) {
			setupNodeDescriptorSet(child);
		}
	}

	void setupDescriptors()
	{
		/*
			Descriptor Pool
		*/
		uint32_t imageSamplerCount = 0;
		uint32_t materialCount = 0;
		uint32_t meshCount = 0;

		// Environment samplers (radiance, irradiance, brdf lut)
		imageSamplerCount += 3;

		std::vector<vkglTF::Model*> modellist = { &models.skybox, &models.scene };
		for (auto &model : modellist) {
		  /* for (auto &material : model->materials) {
			  
				imageSamplerCount += 5;
				materialCount++;
				} */
			imageSamplerCount += 5 * model->materials.size();
			materialCount += model->materials.size();
			
			for (auto node : model->linearNodes) {
				if (node->mesh) {
					meshCount++;
				}
			}
		}

#ifdef WITH_DISPLAY
		int num_images = swapChain.imageCount;
#else // WITH_DISPLAY
		int num_images = 1;
#endif // WITH_DISPLAY

		std::vector<VkDescriptorPoolSize> poolSizes = {
			{ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, (4 + meshCount) * num_images },
			{ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, imageSamplerCount * num_images }
		};

		VkDescriptorPoolCreateInfo descriptorPoolCI{};
		descriptorPoolCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
		descriptorPoolCI.poolSizeCount = 2;
		descriptorPoolCI.pPoolSizes = poolSizes.data();
		descriptorPoolCI.maxSets = (2 + materialCount + meshCount) * num_images;
		VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolCI, nullptr, &descriptorPool));

		/*
			Descriptor sets
		*/

		// Scene (matrices and environment maps)
		{
			std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
				{ 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr },
				{ 1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr },
				{ 2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr },
				{ 3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr },
				{ 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr },
			};
			VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCI{};
			descriptorSetLayoutCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
			descriptorSetLayoutCI.pBindings = setLayoutBindings.data();
			descriptorSetLayoutCI.bindingCount = static_cast<uint32_t>(setLayoutBindings.size());
			VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCI, nullptr, &descriptorSetLayouts.scene));

			for (size_t i = 0; i < descriptorSets.size(); i++) {

				VkDescriptorSetAllocateInfo descriptorSetAllocInfo{};
				descriptorSetAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
				descriptorSetAllocInfo.descriptorPool = descriptorPool;
				descriptorSetAllocInfo.pSetLayouts = &descriptorSetLayouts.scene;
				descriptorSetAllocInfo.descriptorSetCount = 1;
				VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &descriptorSetAllocInfo, &descriptorSets[i].scene));

				std::array<VkWriteDescriptorSet, 5> writeDescriptorSets{};

				writeDescriptorSets[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
				writeDescriptorSets[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
				writeDescriptorSets[0].descriptorCount = 1;
				writeDescriptorSets[0].dstSet = descriptorSets[i].scene;
				writeDescriptorSets[0].dstBinding = 0;
				writeDescriptorSets[0].pBufferInfo = &uniformBuffers[i].scene.descriptor;

				writeDescriptorSets[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
				writeDescriptorSets[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
				writeDescriptorSets[1].descriptorCount = 1;
				writeDescriptorSets[1].dstSet = descriptorSets[i].scene;
				writeDescriptorSets[1].dstBinding = 1;
				writeDescriptorSets[1].pBufferInfo = &uniformBuffers[i].params.descriptor;

				writeDescriptorSets[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
				writeDescriptorSets[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
				writeDescriptorSets[2].descriptorCount = 1;
				writeDescriptorSets[2].dstSet = descriptorSets[i].scene;
				writeDescriptorSets[2].dstBinding = 2;
				writeDescriptorSets[2].pImageInfo = &textures.irradianceCube.descriptor;

				writeDescriptorSets[3].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
				writeDescriptorSets[3].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
				writeDescriptorSets[3].descriptorCount = 1;
				writeDescriptorSets[3].dstSet = descriptorSets[i].scene;
				writeDescriptorSets[3].dstBinding = 3;
				writeDescriptorSets[3].pImageInfo = &textures.prefilteredCube.descriptor;

				writeDescriptorSets[4].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
				writeDescriptorSets[4].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
				writeDescriptorSets[4].descriptorCount = 1;
				writeDescriptorSets[4].dstSet = descriptorSets[i].scene;
				writeDescriptorSets[4].dstBinding = 4;
				writeDescriptorSets[4].pImageInfo = &textures.lutBrdf.descriptor;

				vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, NULL);
			}
		}

		// Material (samplers)
		{
			std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
				{ 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr },
				{ 1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr },
				{ 2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr },
				{ 3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr },
				{ 4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr },
			};
			VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCI{};
			descriptorSetLayoutCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
			descriptorSetLayoutCI.pBindings = setLayoutBindings.data();
			descriptorSetLayoutCI.bindingCount = static_cast<uint32_t>(setLayoutBindings.size());
			VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCI, nullptr, &descriptorSetLayouts.material));

			// Per-Material descriptor sets
			for (auto &material : models.scene.materials) {
				VkDescriptorSetAllocateInfo descriptorSetAllocInfo{};
				descriptorSetAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
				descriptorSetAllocInfo.descriptorPool = descriptorPool;
				descriptorSetAllocInfo.pSetLayouts = &descriptorSetLayouts.material;
				descriptorSetAllocInfo.descriptorSetCount = 1;
				VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &descriptorSetAllocInfo, &material.descriptorSet));

				std::vector<VkDescriptorImageInfo> imageDescriptors = {
					textures.empty.descriptor,
					textures.empty.descriptor,
					material.normalTexture ? material.normalTexture->descriptor : textures.empty.descriptor,
					material.occlusionTexture ? material.occlusionTexture->descriptor : textures.empty.descriptor,
					material.emissiveTexture ? material.emissiveTexture->descriptor : textures.empty.descriptor
				};

				// TODO: glTF specs states that metallic roughness should be preferred, even if specular glosiness is present

				if (material.pbrWorkflows.metallicRoughness) {
					if (material.baseColorTexture) {
						imageDescriptors[0] = material.baseColorTexture->descriptor;
					}
					if (material.metallicRoughnessTexture) {
						imageDescriptors[1] = material.metallicRoughnessTexture->descriptor;
					}
				}

				if (material.pbrWorkflows.specularGlossiness) {
					if (material.extension.diffuseTexture) {
						imageDescriptors[0] = material.extension.diffuseTexture->descriptor;
					}
					if (material.extension.specularGlossinessTexture) {
						imageDescriptors[1] = material.extension.specularGlossinessTexture->descriptor;
					}
				}

				std::array<VkWriteDescriptorSet, 5> writeDescriptorSets{};
				for (size_t i = 0; i < imageDescriptors.size(); i++) {
					writeDescriptorSets[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
					writeDescriptorSets[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
					writeDescriptorSets[i].descriptorCount = 1;
					writeDescriptorSets[i].dstSet = material.descriptorSet;
					writeDescriptorSets[i].dstBinding = static_cast<uint32_t>(i);
					writeDescriptorSets[i].pImageInfo = &imageDescriptors[i];
				}

				vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, NULL);
			}

			// Model node (matrices)
			{
				std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
					{ 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT, nullptr },
				};
				VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCI{};
				descriptorSetLayoutCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
				descriptorSetLayoutCI.pBindings = setLayoutBindings.data();
				descriptorSetLayoutCI.bindingCount = static_cast<uint32_t>(setLayoutBindings.size());
				VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCI, nullptr, &descriptorSetLayouts.node));

				// Per-Node descriptor set
				for (auto &node : models.scene.nodes) {
					setupNodeDescriptorSet(node);
				}
			}

		}

		// Skybox (fixed set)
		for (size_t i = 0; i < uniformBuffers.size(); i++) {
			VkDescriptorSetAllocateInfo descriptorSetAllocInfo{};
			descriptorSetAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
			descriptorSetAllocInfo.descriptorPool = descriptorPool;
			descriptorSetAllocInfo.pSetLayouts = &descriptorSetLayouts.scene;
			descriptorSetAllocInfo.descriptorSetCount = 1;
			VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &descriptorSetAllocInfo, &descriptorSets[i].skybox));

			std::array<VkWriteDescriptorSet, 3> writeDescriptorSets{};

			writeDescriptorSets[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writeDescriptorSets[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
			writeDescriptorSets[0].descriptorCount = 1;
			writeDescriptorSets[0].dstSet = descriptorSets[i].skybox;
			writeDescriptorSets[0].dstBinding = 0;
			writeDescriptorSets[0].pBufferInfo = &uniformBuffers[i].skybox.descriptor;

			writeDescriptorSets[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writeDescriptorSets[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
			writeDescriptorSets[1].descriptorCount = 1;
			writeDescriptorSets[1].dstSet = descriptorSets[i].skybox;
			writeDescriptorSets[1].dstBinding = 1;
			writeDescriptorSets[1].pBufferInfo = &uniformBuffers[i].params.descriptor;

			writeDescriptorSets[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writeDescriptorSets[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
			writeDescriptorSets[2].descriptorCount = 1;
			writeDescriptorSets[2].dstSet = descriptorSets[i].skybox;
			writeDescriptorSets[2].dstBinding = 2;
			writeDescriptorSets[2].pImageInfo = &textures.prefilteredCube.descriptor;

			vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, nullptr);
		}
	}

	void preparePipelines()
	{
		VkPipelineInputAssemblyStateCreateInfo inputAssemblyStateCI{};
		inputAssemblyStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
		inputAssemblyStateCI.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

		VkPipelineRasterizationStateCreateInfo rasterizationStateCI{};
		rasterizationStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
		rasterizationStateCI.polygonMode = VK_POLYGON_MODE_FILL;
		rasterizationStateCI.cullMode = VK_CULL_MODE_BACK_BIT;
		rasterizationStateCI.frontFace = VK_FRONT_FACE_CLOCKWISE;
		rasterizationStateCI.lineWidth = 1.0f;

		VkPipelineColorBlendAttachmentState blendAttachmentState{};
		blendAttachmentState.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
		blendAttachmentState.blendEnable = VK_FALSE;

		VkPipelineColorBlendStateCreateInfo colorBlendStateCI{};
		colorBlendStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
		colorBlendStateCI.attachmentCount = 1;
		colorBlendStateCI.pAttachments = &blendAttachmentState;

		VkPipelineDepthStencilStateCreateInfo depthStencilStateCI{};
		depthStencilStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
		depthStencilStateCI.depthTestEnable = VK_FALSE;
		depthStencilStateCI.depthWriteEnable = VK_FALSE;
		depthStencilStateCI.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
		depthStencilStateCI.front = depthStencilStateCI.back;
		depthStencilStateCI.back.compareOp = VK_COMPARE_OP_ALWAYS;

		VkPipelineViewportStateCreateInfo viewportStateCI{};
		viewportStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
		viewportStateCI.viewportCount = 1;
		viewportStateCI.scissorCount = 1;

		VkPipelineMultisampleStateCreateInfo multisampleStateCI{};
		multisampleStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;

		if (settings.multiSampling) {
		    multisampleStateCI.rasterizationSamples = settings.sampleCount;
		} else {
		    multisampleStateCI.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
		}

		std::vector<VkDynamicState> dynamicStateEnables = {
			VK_DYNAMIC_STATE_VIEWPORT,
			VK_DYNAMIC_STATE_SCISSOR
		};
		VkPipelineDynamicStateCreateInfo dynamicStateCI{};
		dynamicStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
		dynamicStateCI.pDynamicStates = dynamicStateEnables.data();
		dynamicStateCI.dynamicStateCount = static_cast<uint32_t>(dynamicStateEnables.size());

		// Pipeline layout
		const std::vector<VkDescriptorSetLayout> setLayouts = {
			descriptorSetLayouts.scene, descriptorSetLayouts.material, descriptorSetLayouts.node
		};
		VkPipelineLayoutCreateInfo pipelineLayoutCI{};
		pipelineLayoutCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
		pipelineLayoutCI.setLayoutCount = static_cast<uint32_t>(setLayouts.size());
		pipelineLayoutCI.pSetLayouts = setLayouts.data();
		VkPushConstantRange pushConstantRange{};
		pushConstantRange.size = sizeof(PushConstBlockMaterial);
		pushConstantRange.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
		pipelineLayoutCI.pushConstantRangeCount = 1;
		pipelineLayoutCI.pPushConstantRanges = &pushConstantRange;
		VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCI, nullptr, &pipelineLayout));

		// Vertex bindings an attributes
		VkVertexInputBindingDescription vertexInputBinding = { 0, sizeof(vkglTF::Model::Vertex), VK_VERTEX_INPUT_RATE_VERTEX };
		std::vector<VkVertexInputAttributeDescription> vertexInputAttributes = {
			{ 0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0 },
			{ 1, 0, VK_FORMAT_R32G32B32_SFLOAT, sizeof(float) * 3 },
			{ 2, 0, VK_FORMAT_R32G32_SFLOAT, sizeof(float) * 6 },
			{ 3, 0, VK_FORMAT_R32G32_SFLOAT, sizeof(float) * 8 },
			{ 4, 0, VK_FORMAT_R32G32B32A32_SFLOAT, sizeof(float) * 10 },
			{ 5, 0, VK_FORMAT_R32G32B32A32_SFLOAT, sizeof(float) * 14 }
		};
		VkPipelineVertexInputStateCreateInfo vertexInputStateCI{};
		vertexInputStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
		vertexInputStateCI.vertexBindingDescriptionCount = 1;
		vertexInputStateCI.pVertexBindingDescriptions = &vertexInputBinding;
		vertexInputStateCI.vertexAttributeDescriptionCount = static_cast<uint32_t>(vertexInputAttributes.size());
		vertexInputStateCI.pVertexAttributeDescriptions = vertexInputAttributes.data();

		// Pipelines
		std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

		VkGraphicsPipelineCreateInfo pipelineCI{};
		pipelineCI.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
		pipelineCI.layout = pipelineLayout;
		// pipelineCI.renderPass = renderPass; // customStuff.renderPass;
		pipelineCI.renderPass = customStuff.renderPass;
		pipelineCI.pInputAssemblyState = &inputAssemblyStateCI;
		pipelineCI.pVertexInputState = &vertexInputStateCI;
		pipelineCI.pRasterizationState = &rasterizationStateCI;
		pipelineCI.pColorBlendState = &colorBlendStateCI;
		pipelineCI.pMultisampleState = &multisampleStateCI;
		pipelineCI.pViewportState = &viewportStateCI;
		pipelineCI.pDepthStencilState = &depthStencilStateCI;
		pipelineCI.pDynamicState = &dynamicStateCI;
		pipelineCI.stageCount = static_cast<uint32_t>(shaderStages.size());
		pipelineCI.pStages = shaderStages.data();

		if (settings.multiSampling) {
			multisampleStateCI.rasterizationSamples = settings.sampleCount;
		} else {
		    multisampleStateCI.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
		}

		// Skybox pipeline (background cube)
		shaderStages = {
			loadShader(device, "skybox.vert.spv", VK_SHADER_STAGE_VERTEX_BIT),
			loadShader(device, "skybox.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT)
		};

		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.skybox));
		
		for (auto shaderStage : shaderStages) {
			vkDestroyShaderModule(device, shaderStage.module, nullptr);
		}

		// PBR pipeline
		shaderStages = {
			loadShader(device, "pbr.vert.spv", VK_SHADER_STAGE_VERTEX_BIT),
			loadShader(device, "pbr_khr.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT)
		};
		depthStencilStateCI.depthWriteEnable = VK_TRUE;
		depthStencilStateCI.depthTestEnable = VK_TRUE;

		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.pbr));

		rasterizationStateCI.cullMode = VK_CULL_MODE_NONE;
		blendAttachmentState.blendEnable = VK_TRUE;
		blendAttachmentState.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
		blendAttachmentState.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
		blendAttachmentState.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
		blendAttachmentState.colorBlendOp = VK_BLEND_OP_ADD;
		blendAttachmentState.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
		blendAttachmentState.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
		blendAttachmentState.alphaBlendOp = VK_BLEND_OP_ADD;

		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipelines.pbrAlphaBlend));
		

		for (auto shaderStage : shaderStages) {
			vkDestroyShaderModule(device, shaderStage.module, nullptr);
		}
	}

	/*
		Generate a BRDF integration map storing roughness/NdotV as a look-up-table
	*/
	void generateBRDFLUT()
	{
		auto tStart = std::chrono::high_resolution_clock::now();

		const VkFormat format = VK_FORMAT_R16G16_SFLOAT;
		const int32_t dim = 512;

		// Image
		VkImageCreateInfo imageCI{};
		imageCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
		imageCI.imageType = VK_IMAGE_TYPE_2D;
		imageCI.format = format;
		imageCI.extent.width = dim;
		imageCI.extent.height = dim;
		imageCI.extent.depth = 1;
		imageCI.mipLevels = 1;
		imageCI.arrayLayers = 1;
		imageCI.samples = VK_SAMPLE_COUNT_1_BIT;
		imageCI.tiling = VK_IMAGE_TILING_OPTIMAL;
		imageCI.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
		VK_CHECK_RESULT(vkCreateImage(device, &imageCI, nullptr, &textures.lutBrdf.image));
		VkMemoryRequirements memReqs;
		vkGetImageMemoryRequirements(device, textures.lutBrdf.image, &memReqs);
		VkMemoryAllocateInfo memAllocInfo{};
		memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		memAllocInfo.allocationSize = memReqs.size;
		memAllocInfo.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
		VK_CHECK_RESULT(vkAllocateMemory(device, &memAllocInfo, nullptr, &textures.lutBrdf.deviceMemory));
		VK_CHECK_RESULT(vkBindImageMemory(device, textures.lutBrdf.image, textures.lutBrdf.deviceMemory, 0));

		// View
		VkImageViewCreateInfo viewCI{};
		viewCI.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		viewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
		viewCI.format = format;
		viewCI.subresourceRange = {};
		viewCI.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		viewCI.subresourceRange.levelCount = 1;
		viewCI.subresourceRange.layerCount = 1;
		viewCI.image = textures.lutBrdf.image;
		VK_CHECK_RESULT(vkCreateImageView(device, &viewCI, nullptr, &textures.lutBrdf.view));

		// Sampler
		VkSamplerCreateInfo samplerCI{};
		samplerCI.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
		samplerCI.magFilter = VK_FILTER_LINEAR;
		samplerCI.minFilter = VK_FILTER_LINEAR;
		samplerCI.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
		samplerCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samplerCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samplerCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
		samplerCI.minLod = 0.0f;
		samplerCI.maxLod = 1.0f;
		samplerCI.maxAnisotropy = 1.0f;
		samplerCI.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
		VK_CHECK_RESULT(vkCreateSampler(device, &samplerCI, nullptr, &textures.lutBrdf.sampler));

		// FB, Att, RP, Pipe, etc.
		VkAttachmentDescription attDesc{};
		// Color attachment
		attDesc.format = format;
		attDesc.samples = VK_SAMPLE_COUNT_1_BIT;
		attDesc.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		attDesc.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		attDesc.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		attDesc.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		attDesc.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
		attDesc.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		VkAttachmentReference colorReference = { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };

		VkSubpassDescription subpassDescription{};
		subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
		subpassDescription.colorAttachmentCount = 1;
		subpassDescription.pColorAttachments = &colorReference;

		// Use subpass dependencies for layout transitions
		std::array<VkSubpassDependency, 2> dependencies;
		dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[0].dstSubpass = 0;
		dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
		dependencies[1].srcSubpass = 0;
		dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		// Create the actual renderpass
		VkRenderPassCreateInfo renderPassCI{};
		renderPassCI.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
		renderPassCI.attachmentCount = 1;
		renderPassCI.pAttachments = &attDesc;
		renderPassCI.subpassCount = 1;
		renderPassCI.pSubpasses = &subpassDescription;
		renderPassCI.dependencyCount = 2;
		renderPassCI.pDependencies = dependencies.data();

		VkRenderPass renderpass;
		VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassCI, nullptr, &renderpass));

		VkFramebufferCreateInfo framebufferCI{};
		framebufferCI.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		framebufferCI.renderPass = renderpass;
		framebufferCI.attachmentCount = 1;
		framebufferCI.pAttachments = &textures.lutBrdf.view;
		framebufferCI.width = dim;
		framebufferCI.height = dim;
		framebufferCI.layers = 1;

		VkFramebuffer framebuffer;
		VK_CHECK_RESULT(vkCreateFramebuffer(device, &framebufferCI, nullptr, &framebuffer));

		// Desriptors
		VkDescriptorSetLayout descriptorsetlayout;
		VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCI{};
		descriptorSetLayoutCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
		VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCI, nullptr, &descriptorsetlayout));

		// Pipeline layout
		VkPipelineLayout pipelinelayout;
		VkPipelineLayoutCreateInfo pipelineLayoutCI{};
		pipelineLayoutCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
		pipelineLayoutCI.setLayoutCount = 1;
		pipelineLayoutCI.pSetLayouts = &descriptorsetlayout;
		VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCI, nullptr, &pipelinelayout));

		// Pipeline
		VkPipelineInputAssemblyStateCreateInfo inputAssemblyStateCI{};
		inputAssemblyStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
		inputAssemblyStateCI.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

		VkPipelineRasterizationStateCreateInfo rasterizationStateCI{};
		rasterizationStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
		rasterizationStateCI.polygonMode = VK_POLYGON_MODE_FILL;
		rasterizationStateCI.cullMode = VK_CULL_MODE_NONE;
		rasterizationStateCI.frontFace = VK_FRONT_FACE_CLOCKWISE;
		rasterizationStateCI.lineWidth = 1.0f;

		VkPipelineColorBlendAttachmentState blendAttachmentState{};
		blendAttachmentState.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
		blendAttachmentState.blendEnable = VK_FALSE;

		VkPipelineColorBlendStateCreateInfo colorBlendStateCI{};
		colorBlendStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
		colorBlendStateCI.attachmentCount = 1;
		colorBlendStateCI.pAttachments = &blendAttachmentState;

		VkPipelineDepthStencilStateCreateInfo depthStencilStateCI{};
		depthStencilStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
		depthStencilStateCI.depthTestEnable = VK_FALSE;
		depthStencilStateCI.depthWriteEnable = VK_FALSE;
		depthStencilStateCI.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
		depthStencilStateCI.front = depthStencilStateCI.back;
		depthStencilStateCI.back.compareOp = VK_COMPARE_OP_ALWAYS;

		VkPipelineViewportStateCreateInfo viewportStateCI{};
		viewportStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
		viewportStateCI.viewportCount = 1;
		viewportStateCI.scissorCount = 1;

		VkPipelineMultisampleStateCreateInfo multisampleStateCI{};
		multisampleStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
		multisampleStateCI.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

		std::vector<VkDynamicState> dynamicStateEnables = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
		VkPipelineDynamicStateCreateInfo dynamicStateCI{};
		dynamicStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
		dynamicStateCI.pDynamicStates = dynamicStateEnables.data();
		dynamicStateCI.dynamicStateCount = static_cast<uint32_t>(dynamicStateEnables.size());
		
		VkPipelineVertexInputStateCreateInfo emptyInputStateCI{};
		emptyInputStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

		std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

		VkGraphicsPipelineCreateInfo pipelineCI{};
		pipelineCI.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
		pipelineCI.layout = pipelinelayout;
		pipelineCI.renderPass = renderpass;
		pipelineCI.pInputAssemblyState = &inputAssemblyStateCI;
		pipelineCI.pVertexInputState = &emptyInputStateCI;
		pipelineCI.pRasterizationState = &rasterizationStateCI;
		pipelineCI.pColorBlendState = &colorBlendStateCI;
		pipelineCI.pMultisampleState = &multisampleStateCI;
		pipelineCI.pViewportState = &viewportStateCI;
		pipelineCI.pDepthStencilState = &depthStencilStateCI;
		pipelineCI.pDynamicState = &dynamicStateCI;
		pipelineCI.stageCount = 2;
		pipelineCI.pStages = shaderStages.data();

		// Look-up-table (from BRDF) pipeline		
		shaderStages = {
			loadShader(device, "genbrdflut.vert.spv", VK_SHADER_STAGE_VERTEX_BIT),
			loadShader(device, "genbrdflut.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT)
		};
		VkPipeline pipeline;
		VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipeline));
		for (auto shaderStage : shaderStages) {
			vkDestroyShaderModule(device, shaderStage.module, nullptr);
		}

		// Render
		VkClearValue clearValues[1];
		clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 1.0f } };

		VkRenderPassBeginInfo renderPassBeginInfo{};
		renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
		renderPassBeginInfo.renderPass = renderpass;
		renderPassBeginInfo.renderArea.extent.width = dim;
		renderPassBeginInfo.renderArea.extent.height = dim;
		renderPassBeginInfo.clearValueCount = 1;
		renderPassBeginInfo.pClearValues = clearValues;
		renderPassBeginInfo.framebuffer = framebuffer;

		VkCommandBuffer cmdBuf = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);
		vkCmdBeginRenderPass(cmdBuf, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

		VkViewport viewport{};
		viewport.width = (float)dim;
		viewport.height = (float)dim;
		viewport.minDepth = 0.0f;
		viewport.maxDepth = 1.0f;

		VkRect2D scissor{};
		scissor.extent.width = dim;
		scissor.extent.height = dim;

		vkCmdSetViewport(cmdBuf, 0, 1, &viewport);
		vkCmdSetScissor(cmdBuf, 0, 1, &scissor);
		vkCmdBindPipeline(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
		vkCmdDraw(cmdBuf, 3, 1, 0, 0);
		vkCmdEndRenderPass(cmdBuf);
		vulkanDevice->flushCommandBuffer(cmdBuf, queue);

		vkQueueWaitIdle(queue);

		vkDestroyPipeline(device, pipeline, nullptr);
		vkDestroyPipelineLayout(device, pipelinelayout, nullptr);
		vkDestroyRenderPass(device, renderpass, nullptr);
		vkDestroyFramebuffer(device, framebuffer, nullptr);
		vkDestroyDescriptorSetLayout(device, descriptorsetlayout, nullptr);

		textures.lutBrdf.descriptor.imageView = textures.lutBrdf.view;
		textures.lutBrdf.descriptor.sampler = textures.lutBrdf.sampler;
		textures.lutBrdf.descriptor.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
		textures.lutBrdf.device = vulkanDevice;

		auto tEnd = std::chrono::high_resolution_clock::now();
		auto tDiff = std::chrono::duration<double, std::milli>(tEnd - tStart).count();
		std::cout << "Generating BRDF LUT took " << tDiff << " ms" << std::endl;
	}

	/*
		Offline generation for the cube maps used for PBR lighting		
		- Irradiance cube map
		- Pre-filterd environment cubemap
	*/
	void generateCubemaps()
	{
		enum Target { IRRADIANCE = 0, PREFILTEREDENV = 1 };

		for (uint32_t target = 0; target < PREFILTEREDENV + 1; target++) {

			vks::TextureCubeMap cubemap;

			auto tStart = std::chrono::high_resolution_clock::now();

			VkFormat format;
			int32_t dim;

			switch (target) {
			case IRRADIANCE:
				format = VK_FORMAT_R32G32B32A32_SFLOAT;
				dim = 64;
				break;
			case PREFILTEREDENV:
				format = VK_FORMAT_R16G16B16A16_SFLOAT;
				dim = 512;
				break;
			};

			const uint32_t numMips = static_cast<uint32_t>(floor(log2(dim))) + 1;

			// Create target cubemap
			{
				// Image
				VkImageCreateInfo imageCI{};
				imageCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
				imageCI.imageType = VK_IMAGE_TYPE_2D;
				imageCI.format = format;
				imageCI.extent.width = dim;
				imageCI.extent.height = dim;
				imageCI.extent.depth = 1;
				imageCI.mipLevels = numMips;
				imageCI.arrayLayers = 6;
				imageCI.samples = VK_SAMPLE_COUNT_1_BIT;
				imageCI.tiling = VK_IMAGE_TILING_OPTIMAL;
				imageCI.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
				imageCI.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
				VK_CHECK_RESULT(vkCreateImage(device, &imageCI, nullptr, &cubemap.image));
				VkMemoryRequirements memReqs;
				vkGetImageMemoryRequirements(device, cubemap.image, &memReqs);
				VkMemoryAllocateInfo memAllocInfo{};
				memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
				memAllocInfo.allocationSize = memReqs.size;
				memAllocInfo.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
				VK_CHECK_RESULT(vkAllocateMemory(device, &memAllocInfo, nullptr, &cubemap.deviceMemory));
				VK_CHECK_RESULT(vkBindImageMemory(device, cubemap.image, cubemap.deviceMemory, 0));

				// View
				VkImageViewCreateInfo viewCI{};
				viewCI.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
				viewCI.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
				viewCI.format = format;
				viewCI.subresourceRange = {};
				viewCI.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
				viewCI.subresourceRange.levelCount = numMips;
				viewCI.subresourceRange.layerCount = 6;
				viewCI.image = cubemap.image;
				VK_CHECK_RESULT(vkCreateImageView(device, &viewCI, nullptr, &cubemap.view));

				// Sampler
				VkSamplerCreateInfo samplerCI{};
				samplerCI.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
				samplerCI.magFilter = VK_FILTER_LINEAR;
				samplerCI.minFilter = VK_FILTER_LINEAR;
				samplerCI.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
				samplerCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
				samplerCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
				samplerCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
				samplerCI.minLod = 0.0f;
				samplerCI.maxLod = static_cast<float>(numMips);
				samplerCI.maxAnisotropy = 1.0f;
				samplerCI.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
				VK_CHECK_RESULT(vkCreateSampler(device, &samplerCI, nullptr, &cubemap.sampler));

			}

			// FB, Att, RP, Pipe, etc.
			VkAttachmentDescription attDesc{};
			// Color attachment
			attDesc.format = format;
			attDesc.samples = VK_SAMPLE_COUNT_1_BIT;
			attDesc.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
			attDesc.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
			attDesc.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
			attDesc.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
			attDesc.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			attDesc.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
			VkAttachmentReference colorReference = { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };

			VkSubpassDescription subpassDescription{};
			subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
			subpassDescription.colorAttachmentCount = 1;
			subpassDescription.pColorAttachments = &colorReference;

			// Use subpass dependencies for layout transitions
			std::array<VkSubpassDependency, 2> dependencies;
			dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
			dependencies[0].dstSubpass = 0;
			dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
			dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
			dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
			dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
			dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
			dependencies[1].srcSubpass = 0;
			dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
			dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
			dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
			dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
			dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
			dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

			// Renderpass
			VkRenderPassCreateInfo renderPassCI{};
			renderPassCI.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
			renderPassCI.attachmentCount = 1;
			renderPassCI.pAttachments = &attDesc;
			renderPassCI.subpassCount = 1;
			renderPassCI.pSubpasses = &subpassDescription;
			renderPassCI.dependencyCount = 2;
			renderPassCI.pDependencies = dependencies.data();
			VkRenderPass renderpass;
			VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassCI, nullptr, &renderpass));

			struct Offscreen {
				VkImage image;
				VkImageView view;
				VkDeviceMemory memory;
				VkFramebuffer framebuffer;
			} offscreen;

			// Create offscreen framebuffer
			{
				// Image
				VkImageCreateInfo imageCI{};
				imageCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
				imageCI.imageType = VK_IMAGE_TYPE_2D;
				imageCI.format = format;
				imageCI.extent.width = dim;
				imageCI.extent.height = dim;
				imageCI.extent.depth = 1;
				imageCI.mipLevels = 1;
				imageCI.arrayLayers = 1;
				imageCI.samples = VK_SAMPLE_COUNT_1_BIT;
				imageCI.tiling = VK_IMAGE_TILING_OPTIMAL;
				imageCI.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				imageCI.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
				imageCI.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
				VK_CHECK_RESULT(vkCreateImage(device, &imageCI, nullptr, &offscreen.image));
				VkMemoryRequirements memReqs;
				vkGetImageMemoryRequirements(device, offscreen.image, &memReqs);
				VkMemoryAllocateInfo memAllocInfo{};
				memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
				memAllocInfo.allocationSize = memReqs.size;
				memAllocInfo.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
				VK_CHECK_RESULT(vkAllocateMemory(device, &memAllocInfo, nullptr, &offscreen.memory));
				VK_CHECK_RESULT(vkBindImageMemory(device, offscreen.image, offscreen.memory, 0));

				// View
				VkImageViewCreateInfo viewCI{};
				viewCI.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
				viewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
				viewCI.format = format;
				viewCI.flags = 0;
				viewCI.subresourceRange = {};
				viewCI.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
				viewCI.subresourceRange.baseMipLevel = 0;
				viewCI.subresourceRange.levelCount = 1;
				viewCI.subresourceRange.baseArrayLayer = 0;
				viewCI.subresourceRange.layerCount = 1;
				viewCI.image = offscreen.image;
				VK_CHECK_RESULT(vkCreateImageView(device, &viewCI, nullptr, &offscreen.view));

				// Framebuffer
				VkFramebufferCreateInfo framebufferCI{};
				framebufferCI.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
				framebufferCI.renderPass = renderpass;
				framebufferCI.attachmentCount = 1;
				framebufferCI.pAttachments = &offscreen.view;
				framebufferCI.width = dim;
				framebufferCI.height = dim;
				framebufferCI.layers = 1;
				VK_CHECK_RESULT(vkCreateFramebuffer(device, &framebufferCI, nullptr, &offscreen.framebuffer));

				VkCommandBuffer layoutCmd = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);
				VkImageMemoryBarrier imageMemoryBarrier{};
				imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
				imageMemoryBarrier.image = offscreen.image;
				imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
				imageMemoryBarrier.srcAccessMask = 0;
				imageMemoryBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
				imageMemoryBarrier.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
				vkCmdPipelineBarrier(layoutCmd, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &imageMemoryBarrier);
				vulkanDevice->flushCommandBuffer(layoutCmd, queue, true);
			}

			// Descriptors
			VkDescriptorSetLayout descriptorsetlayout;
			VkDescriptorSetLayoutBinding setLayoutBinding = { 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr };
			VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCI{};
			descriptorSetLayoutCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
			descriptorSetLayoutCI.pBindings = &setLayoutBinding;
			descriptorSetLayoutCI.bindingCount = 1;
			VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorSetLayoutCI, nullptr, &descriptorsetlayout));

			// Descriptor Pool
			VkDescriptorPoolSize poolSize = { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1 };
			VkDescriptorPoolCreateInfo descriptorPoolCI{};
			descriptorPoolCI.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
			descriptorPoolCI.poolSizeCount = 1;
			descriptorPoolCI.pPoolSizes = &poolSize;
			descriptorPoolCI.maxSets = 2;
			VkDescriptorPool descriptorpool;
			VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolCI, nullptr, &descriptorpool));

			// Descriptor sets
			VkDescriptorSet descriptorset;
			VkDescriptorSetAllocateInfo descriptorSetAllocInfo{};
			descriptorSetAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
			descriptorSetAllocInfo.descriptorPool = descriptorpool;
			descriptorSetAllocInfo.pSetLayouts = &descriptorsetlayout;
			descriptorSetAllocInfo.descriptorSetCount = 1;
			VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &descriptorSetAllocInfo, &descriptorset));
			VkWriteDescriptorSet writeDescriptorSet{};
			writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			writeDescriptorSet.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
			writeDescriptorSet.descriptorCount = 1;
			writeDescriptorSet.dstSet = descriptorset;
			writeDescriptorSet.dstBinding = 0;
			writeDescriptorSet.pImageInfo = &textures.environmentCube.descriptor;

			vkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, nullptr);

			struct PushBlockIrradiance {
				glm::mat4 mvp;
				float deltaPhi = (2.0f * float(M_PI)) / 180.0f;
				float deltaTheta = (0.5f * float(M_PI)) / 64.0f;
			} pushBlockIrradiance;

			struct PushBlockPrefilterEnv {
				glm::mat4 mvp;
				float roughness;
				uint32_t numSamples = 32u;
			} pushBlockPrefilterEnv;

			// Pipeline layout
			VkPipelineLayout pipelinelayout;
			VkPushConstantRange pushConstantRange{};
			pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

			switch (target) {
				case IRRADIANCE:
					pushConstantRange.size = sizeof(PushBlockIrradiance);
					break;
				case PREFILTEREDENV:
					pushConstantRange.size = sizeof(PushBlockPrefilterEnv);
					break;
			};

			VkPipelineLayoutCreateInfo pipelineLayoutCI{};
			pipelineLayoutCI.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
			pipelineLayoutCI.setLayoutCount = 1;
			pipelineLayoutCI.pSetLayouts = &descriptorsetlayout;
			pipelineLayoutCI.pushConstantRangeCount = 1;
			pipelineLayoutCI.pPushConstantRanges = &pushConstantRange;
			VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCI, nullptr, &pipelinelayout));

			// Pipeline
			VkPipelineInputAssemblyStateCreateInfo inputAssemblyStateCI{};
			inputAssemblyStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
			inputAssemblyStateCI.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

			VkPipelineRasterizationStateCreateInfo rasterizationStateCI{};
			rasterizationStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
			rasterizationStateCI.polygonMode = VK_POLYGON_MODE_FILL;
			rasterizationStateCI.cullMode = VK_CULL_MODE_NONE;
			rasterizationStateCI.frontFace = VK_FRONT_FACE_CLOCKWISE;
			rasterizationStateCI.lineWidth = 1.0f;

			VkPipelineColorBlendAttachmentState blendAttachmentState{};
			blendAttachmentState.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
			blendAttachmentState.blendEnable = VK_FALSE;

			VkPipelineColorBlendStateCreateInfo colorBlendStateCI{};
			colorBlendStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
			colorBlendStateCI.attachmentCount = 1;
			colorBlendStateCI.pAttachments = &blendAttachmentState;

			VkPipelineDepthStencilStateCreateInfo depthStencilStateCI{};
			depthStencilStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
			depthStencilStateCI.depthTestEnable = VK_FALSE;
			depthStencilStateCI.depthWriteEnable = VK_FALSE;
			depthStencilStateCI.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
			depthStencilStateCI.front = depthStencilStateCI.back;
			depthStencilStateCI.back.compareOp = VK_COMPARE_OP_ALWAYS;

			VkPipelineViewportStateCreateInfo viewportStateCI{};
			viewportStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
			viewportStateCI.viewportCount = 1;
			viewportStateCI.scissorCount = 1;

			VkPipelineMultisampleStateCreateInfo multisampleStateCI{};
			multisampleStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
			multisampleStateCI.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
			
			std::vector<VkDynamicState> dynamicStateEnables = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
			VkPipelineDynamicStateCreateInfo dynamicStateCI{};
			dynamicStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
			dynamicStateCI.pDynamicStates = dynamicStateEnables.data();
			dynamicStateCI.dynamicStateCount = static_cast<uint32_t>(dynamicStateEnables.size());

			// Vertex input state
			VkVertexInputBindingDescription vertexInputBinding = { 0, sizeof(vkglTF::Model::Vertex), VK_VERTEX_INPUT_RATE_VERTEX };
			VkVertexInputAttributeDescription vertexInputAttribute = { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0 };

			VkPipelineVertexInputStateCreateInfo vertexInputStateCI{};
			vertexInputStateCI.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
			vertexInputStateCI.vertexBindingDescriptionCount = 1;
			vertexInputStateCI.pVertexBindingDescriptions = &vertexInputBinding;
			vertexInputStateCI.vertexAttributeDescriptionCount = 1;
			vertexInputStateCI.pVertexAttributeDescriptions = &vertexInputAttribute;

			std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

			VkGraphicsPipelineCreateInfo pipelineCI{};
			pipelineCI.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
			pipelineCI.layout = pipelinelayout;
			pipelineCI.renderPass = renderpass;
			pipelineCI.pInputAssemblyState = &inputAssemblyStateCI;
			pipelineCI.pVertexInputState = &vertexInputStateCI;
			pipelineCI.pRasterizationState = &rasterizationStateCI;
			pipelineCI.pColorBlendState = &colorBlendStateCI;
			pipelineCI.pMultisampleState = &multisampleStateCI;
			pipelineCI.pViewportState = &viewportStateCI;
			pipelineCI.pDepthStencilState = &depthStencilStateCI;
			pipelineCI.pDynamicState = &dynamicStateCI;
			pipelineCI.stageCount = 2;
			pipelineCI.pStages = shaderStages.data();
			pipelineCI.renderPass = renderpass;

			shaderStages[0] = loadShader(device, "filtercube.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
			switch (target) {
				case IRRADIANCE:
					shaderStages[1] = loadShader(device, "irradiancecube.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
					break;
				case PREFILTEREDENV:
					shaderStages[1] = loadShader(device, "prefilterenvmap.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
					break;
			};
			VkPipeline pipeline;
			VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipeline));
			
			for (auto shaderStage : shaderStages) {
				vkDestroyShaderModule(device, shaderStage.module, nullptr);
			}

			// Render cubemap
			VkClearValue clearValues[1];
			clearValues[0].color = { { 0.0f, 0.0f, 0.2f, 0.0f } };

			VkRenderPassBeginInfo renderPassBeginInfo{};
			renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
			renderPassBeginInfo.renderPass = renderpass;
			renderPassBeginInfo.framebuffer = offscreen.framebuffer;
			renderPassBeginInfo.renderArea.extent.width = dim;
			renderPassBeginInfo.renderArea.extent.height = dim;
			renderPassBeginInfo.clearValueCount = 1;
			renderPassBeginInfo.pClearValues = clearValues;

			std::vector<glm::mat4> matrices = {
				glm::rotate(glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
				glm::rotate(glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
				glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
				glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
				glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
				glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 1.0f)),
			};

			VkCommandBuffer cmdBuf = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, false);

			VkViewport viewport{};
			viewport.width = (float)dim;
			viewport.height = (float)dim;
			viewport.minDepth = 0.0f;
			viewport.maxDepth = 1.0f;

			VkRect2D scissor{};
			scissor.extent.width = dim;
			scissor.extent.height = dim;

			VkImageSubresourceRange subresourceRange{};
			subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			subresourceRange.baseMipLevel = 0;
			subresourceRange.levelCount = numMips;
			subresourceRange.layerCount = 6;

			// Change image layout for all cubemap faces to transfer destination
			{
				vulkanDevice->beginCommandBuffer(cmdBuf);
				VkImageMemoryBarrier imageMemoryBarrier{};
				imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
				imageMemoryBarrier.image = cubemap.image;
				imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
				imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
				imageMemoryBarrier.srcAccessMask = 0;
				imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
				imageMemoryBarrier.subresourceRange = subresourceRange;
				vkCmdPipelineBarrier(cmdBuf, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &imageMemoryBarrier);
				vulkanDevice->flushCommandBuffer(cmdBuf, queue, false);
			}

			for (uint32_t m = 0; m < numMips; m++) {
				for (uint32_t f = 0; f < 6; f++) {

					vulkanDevice->beginCommandBuffer(cmdBuf);

					viewport.width = static_cast<float>(dim * std::pow(0.5f, m));
					viewport.height = static_cast<float>(dim * std::pow(0.5f, m));
					vkCmdSetViewport(cmdBuf, 0, 1, &viewport);
					vkCmdSetScissor(cmdBuf, 0, 1, &scissor);

					// Render scene from cube face's point of view
					
					vkCmdBeginRenderPass(cmdBuf, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
					

					// Pass parameters for current pass using a push constant block
					switch (target) {
						case IRRADIANCE:
							pushBlockIrradiance.mvp = glm::perspective((float)(M_PI / 2.0), 1.0f, 0.1f, 512.0f) * matrices[f];
							vkCmdPushConstants(cmdBuf, pipelinelayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushBlockIrradiance), &pushBlockIrradiance);
							break;
						case PREFILTEREDENV:
							pushBlockPrefilterEnv.mvp = glm::perspective((float)(M_PI / 2.0), 1.0f, 0.1f, 512.0f) * matrices[f];
							pushBlockPrefilterEnv.roughness = (float)m / (float)(numMips - 1);
							vkCmdPushConstants(cmdBuf, pipelinelayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushBlockPrefilterEnv), &pushBlockPrefilterEnv);
							break;
					};

					vkCmdBindPipeline(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
					vkCmdBindDescriptorSets(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelinelayout, 0, 1, &descriptorset, 0, NULL);

					models.skybox.draw(cmdBuf);

					vkCmdEndRenderPass(cmdBuf);

					subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
					subresourceRange.baseMipLevel = 0;
					subresourceRange.levelCount = numMips;
					subresourceRange.layerCount = 6;

					{
						VkImageMemoryBarrier imageMemoryBarrier{};
						imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
						imageMemoryBarrier.image = offscreen.image;
						imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
						imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
						imageMemoryBarrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
						imageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
						imageMemoryBarrier.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
						vkCmdPipelineBarrier(cmdBuf, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &imageMemoryBarrier);
					}

					// Copy region for transfer from framebuffer to cube face
					VkImageCopy copyRegion{};

					copyRegion.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
					copyRegion.srcSubresource.baseArrayLayer = 0;
					copyRegion.srcSubresource.mipLevel = 0;
					copyRegion.srcSubresource.layerCount = 1;
					copyRegion.srcOffset = { 0, 0, 0 };

					copyRegion.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
					copyRegion.dstSubresource.baseArrayLayer = f;
					copyRegion.dstSubresource.mipLevel = m;
					copyRegion.dstSubresource.layerCount = 1;
					copyRegion.dstOffset = { 0, 0, 0 };

					copyRegion.extent.width = static_cast<uint32_t>(viewport.width);
					copyRegion.extent.height = static_cast<uint32_t>(viewport.height);
					copyRegion.extent.depth = 1;

					vkCmdCopyImage(
						cmdBuf,
						offscreen.image,
						VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
						cubemap.image,
						VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
						1,
						&copyRegion);

					{
						VkImageMemoryBarrier imageMemoryBarrier{};
						imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
						imageMemoryBarrier.image = offscreen.image;
						imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
						imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
						imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
						imageMemoryBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
						imageMemoryBarrier.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
						vkCmdPipelineBarrier(cmdBuf, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &imageMemoryBarrier);
					}

					vulkanDevice->flushCommandBuffer(cmdBuf, queue, false);
				}
			}

			{
				vulkanDevice->beginCommandBuffer(cmdBuf);
				VkImageMemoryBarrier imageMemoryBarrier{};
				imageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
				imageMemoryBarrier.image = cubemap.image;
				imageMemoryBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
				imageMemoryBarrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
				imageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
				imageMemoryBarrier.dstAccessMask = VK_ACCESS_HOST_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
				imageMemoryBarrier.subresourceRange = subresourceRange;
				vkCmdPipelineBarrier(cmdBuf, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &imageMemoryBarrier);
				vulkanDevice->flushCommandBuffer(cmdBuf, queue, false);
			}


			vkDestroyRenderPass(device, renderpass, nullptr);
			vkDestroyFramebuffer(device, offscreen.framebuffer, nullptr);
			vkFreeMemory(device, offscreen.memory, nullptr);
			vkDestroyImageView(device, offscreen.view, nullptr);
			vkDestroyImage(device, offscreen.image, nullptr);
			vkDestroyDescriptorPool(device, descriptorpool, nullptr);
			vkDestroyDescriptorSetLayout(device, descriptorsetlayout, nullptr);
			vkDestroyPipeline(device, pipeline, nullptr);
			vkDestroyPipelineLayout(device, pipelinelayout, nullptr);

			cubemap.descriptor.imageView = cubemap.view;
			cubemap.descriptor.sampler = cubemap.sampler;
			cubemap.descriptor.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			cubemap.device = vulkanDevice;

			switch (target) {
				case IRRADIANCE:
					textures.irradianceCube = cubemap;
					break;
				case PREFILTEREDENV:
					textures.prefilteredCube = cubemap;
					shaderValuesParams.prefilteredCubeMipLevels = static_cast<float>(numMips);
					break;
			};

			auto tEnd = std::chrono::high_resolution_clock::now();
			auto tDiff = std::chrono::duration<double, std::milli>(tEnd - tStart).count();
			std::cout << "Generating cube map with " << numMips << " mip levels took " << tDiff << " ms" << std::endl;
		}
	}

	/* 
		Prepare and initialize uniform buffers containing shader parameters
	*/
	void prepareUniformBuffers()
	{
		for (auto &uniformBuffer : uniformBuffers) {
			uniformBuffer.scene.create(vulkanDevice, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, sizeof(shaderValuesScene));
			uniformBuffer.skybox.create(vulkanDevice, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, sizeof(shaderValuesSkybox));
			uniformBuffer.params.create(vulkanDevice, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, sizeof(shaderValuesParams));
		}
		updateUniformBuffers();
	}

	void updateUniformBuffers()
	{
		// Scene
		shaderValuesScene.projection = camera.matrices.perspective;
		shaderValuesScene.view = camera.matrices.view;
		
		// Center and scale model
		// float scale = (1.0f / std::max(models.scene.aabb[0][0], std::max(models.scene.aabb[1][1], models.scene.aabb[2][2]))) * 0.5f;
		// Nope
		float scale = 1.0f;
		// And nope
		//glm::vec3 translate = -glm::vec3(models.scene.aabb[3][0], models.scene.aabb[3][1], models.scene.aabb[3][2]);
		// translate += -0.5f * glm::vec3(models.scene.aabb[0][0], models.scene.aabb[1][1], models.scene.aabb[2][2]);
		glm::vec3 translate = glm::vec3(0.0f);

		shaderValuesScene.model = glm::mat4(1.0f);
		shaderValuesScene.model[0][0] = scale; // Mirror fix
		shaderValuesScene.model[1][1] = scale;
		shaderValuesScene.model[2][2] = scale; // Se if we can fix mirroring issue
		shaderValuesScene.model = glm::translate(shaderValuesScene.model, translate);

		shaderValuesScene.camPos = glm::vec3(
			camera.position.z * sin(glm::radians(camera.rotation.y)) * cos(glm::radians(camera.rotation.x)),
			-camera.position.z * sin(glm::radians(camera.rotation.x)),
			-camera.position.z * cos(glm::radians(camera.rotation.y)) * cos(glm::radians(camera.rotation.x))
		);

		// Skybox
		shaderValuesSkybox.projection = camera.matrices.perspective;
		shaderValuesSkybox.view = camera.matrices.view;
		shaderValuesSkybox.model = glm::mat4(glm::mat3(camera.matrices.view));
	}

	void updateParams()
	{
		shaderValuesParams.lightDir = glm::vec4(
			sin(glm::radians(lightSource.rotation.x)) * cos(glm::radians(lightSource.rotation.y)),
			sin(glm::radians(lightSource.rotation.y)),
			cos(glm::radians(lightSource.rotation.x)) * cos(glm::radians(lightSource.rotation.y)),
			0.0f);
	}

#ifdef WITH_DISPLAY
	void windowResized()
	{
		recordCommandBuffers();
		vkDeviceWaitIdle(device);
		updateUniformBuffers();
		updateOverlay();
	}
#endif // WITH_DISPLAY

	void prepare()
	{
		VulkanExampleBase::prepare();

		// camera.type = Camera::CameraType::lookat;
		camera.type = Camera::CameraType::firstperson;

		camera.setPerspective(45.0f, (float)width / (float)height, 0.001f, 256.0f);
		camera.rotationSpeed = 0.25f;
		camera.movementSpeed = 0.1f;
		camera.setPosition({ 0.0f, 0.0f, 1.0f });
		camera.setRotation({ 0.0f, 0.0f, 0.0f });

		waitFences.resize(renderAhead);
		presentCompleteSemaphores.resize(renderAhead);
		renderCompleteSemaphores.resize(renderAhead);

#ifdef WITH_DISPLAY
		int num_images = swapChain.imageCount;
#else // WITH_DISPLAY
		int num_images = 1;
#endif // WITH_DISPLAY
		
		commandBuffers.resize(num_images);
		uniformBuffers.resize(num_images);
		descriptorSets.resize(num_images);
		
		// Command buffer execution fences
		for (auto &waitFence : waitFences) {
			VkFenceCreateInfo fenceCI{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO, nullptr, VK_FENCE_CREATE_SIGNALED_BIT };
			VK_CHECK_RESULT(vkCreateFence(device, &fenceCI, nullptr, &waitFence));
		}
		// Queue ordering semaphores
		for (auto &semaphore : presentCompleteSemaphores) {
			VkSemaphoreCreateInfo semaphoreCI{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO, nullptr, 0 };
			VK_CHECK_RESULT(vkCreateSemaphore(device, &semaphoreCI, nullptr, &semaphore));
		}
		for (auto &semaphore : renderCompleteSemaphores) {
			VkSemaphoreCreateInfo semaphoreCI{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO, nullptr, 0 };
			VK_CHECK_RESULT(vkCreateSemaphore(device, &semaphoreCI, nullptr, &semaphore));
		}
		// Command buffers
		{
			VkCommandBufferAllocateInfo cmdBufAllocateInfo{};
			cmdBufAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
			cmdBufAllocateInfo.commandPool = cmdPool;
			cmdBufAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
			cmdBufAllocateInfo.commandBufferCount = static_cast<uint32_t>(commandBuffers.size());
			VK_CHECK_RESULT(vkAllocateCommandBuffers(device, &cmdBufAllocateInfo, commandBuffers.data()));
		}

		loadAssets();
		generateBRDFLUT();
		prepareUniformBuffers();
		setupDescriptors();
		setupCustomStuff();

		
		preparePipelines();

#ifdef WITH_DISPLAY
		ui = new UI(vulkanDevice, renderPass, queue, pipelineCache, settings.sampleCount);
		updateOverlay();
#endif // WITH_DISPLAY

		recordCommandBuffers();
	        
		
		prepared = true;
	}

    void cmdSetLayout(VkCommandBuffer cmd, VkImage image, VkImageAspectFlags aspect,
				 VkImageLayout oldLayout, VkImageLayout newLayout){
	//Shameful copy-paste from LunarG. I'm sorry :(

	VkImageMemoryBarrier image_memory_barrier = {};
	image_memory_barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	image_memory_barrier.pNext = NULL;
	image_memory_barrier.srcAccessMask = 0;
	image_memory_barrier.dstAccessMask = 0;
	image_memory_barrier.oldLayout = oldLayout;
	image_memory_barrier.newLayout = newLayout;
	image_memory_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	image_memory_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	image_memory_barrier.image = image;
	image_memory_barrier.subresourceRange.aspectMask = aspect;
	image_memory_barrier.subresourceRange.baseMipLevel = 0;
	image_memory_barrier.subresourceRange.levelCount = 1;
	image_memory_barrier.subresourceRange.baseArrayLayer = 0;
	image_memory_barrier.subresourceRange.layerCount = 1;

	VkPipelineStageFlags srcStages, destStages;

	switch (oldLayout) {
	case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
	    image_memory_barrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	    srcStages = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	    break;

	case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
	    image_memory_barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
	    srcStages = VK_PIPELINE_STAGE_TRANSFER_BIT;
	    break;

	case VK_IMAGE_LAYOUT_PREINITIALIZED:
	    image_memory_barrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
	    srcStages = VK_PIPELINE_STAGE_HOST_BIT;
	    break;

	case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
	    image_memory_barrier.srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
	    srcStages = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
	    break;

	case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
	    image_memory_barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	    srcStages = VK_PIPELINE_STAGE_TRANSFER_BIT;
	    break;

	default:
	    srcStages = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
	    break;
	}

	switch (newLayout) {
	case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
	    image_memory_barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
	    destStages = VK_PIPELINE_STAGE_TRANSFER_BIT;
	    break;

	case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
	    image_memory_barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
	    destStages = VK_PIPELINE_STAGE_TRANSFER_BIT;
	    break;

	case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
	    image_memory_barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
	    destStages = VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
	    break;

	case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
	    image_memory_barrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	    destStages = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	    break;

	case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
	    image_memory_barrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
	    destStages = VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
	    break;

	default:
	    destStages = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
	    break;
	}

	// Test this for safety:
	srcStages = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
	destStages = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
	vkCmdPipelineBarrier(cmd, srcStages, destStages, 0, 0, NULL, 0, NULL, 1, &image_memory_barrier);
    }

  void renderCustom(int count, int feature_index) {
      
	if(!settings.followPath) {
	    return;
	}

	/* // See if we can aggrevate some vulkan debug layer output
	VkBuffer bbb;
	VkBufferCreateInfo bci;
	bci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO; // Should give fault
	bci.size = 4;
	bci.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
	bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	if(vkCreateBuffer(device, &bci, nullptr, &bbb) != VK_SUCCESS) {
	  std::cerr << "An error a day... " << std::endl;
	  exit(-1);
	  } */
	// Conclusion: Yes we can
	
	// Set camera perspective aspect to conform to draw dimensions
	// camera.setPerspective(45.0, float(this->width) / this->height, 0.001f, 256.0f);
	// updateUniformBuffers();

	// Submit already-recorded-command
	const VkPipelineStageFlags waitDstStageMask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
	VkSubmitInfo si {};
	si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	si.pWaitDstStageMask = &waitDstStageMask;
	// si.pWaitSemaphores = NULL;
	si.waitSemaphoreCount = 0;
	// si.pWaitSemaphores = &customStuff.copiedSemaphore;
	// si.waitSemaphoreCount = 1;
	// si.pSignalSemaphores = NULL;
	// si.signalSemaphoreCount = 0;
	si.pSignalSemaphores = &customStuff.renderedSemaphore;
	si.signalSemaphoreCount = 1;
	si.pCommandBuffers = &customStuff.commandBuffers[currentBuffer];
	si.commandBufferCount = 1;

	VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &si, customStuff.fence));

	usleep(100000);
	
	VkCommandBufferBeginInfo cmd_begin = {};
	cmd_begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	cmd_begin.pNext = NULL;
	cmd_begin.flags = 0;
	cmd_begin.pInheritanceInfo = NULL;
  
	vkBeginCommandBuffer(customStuff.secondCommandBuffer, &cmd_begin);

	cmdSetLayout(customStuff.secondCommandBuffer, customStuff.fbColor.image, VK_IMAGE_ASPECT_COLOR_BIT,
		     // VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
		     VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
	cmdSetLayout(customStuff.secondCommandBuffer, customStuff.reachableImage.image, VK_IMAGE_ASPECT_COLOR_BIT,
		     VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);


	VkImageCopy ic;
	ic.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	ic.srcSubresource.mipLevel = 0;
	ic.srcSubresource.baseArrayLayer = 0;
	ic.srcSubresource.layerCount = 1;
	ic.srcOffset.x = 0;
	ic.srcOffset.y = 0;
	ic.srcOffset.z = 0;
	ic.extent.width = this->width;
	ic.extent.height = this->height;
	ic.extent.depth = 1;
	
	ic.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	ic.dstSubresource.mipLevel = 0;
	ic.dstSubresource.baseArrayLayer = 0;
	ic.dstSubresource.layerCount = 1;
	ic.dstOffset.x = 0;
	ic.dstOffset.y = 0;
	ic.dstOffset.z = 0;
	ic.extent.width = this->width;
	ic.extent.height = this->height;
	ic.extent.depth = 1;

	vkCmdCopyImage(customStuff.secondCommandBuffer, customStuff.fbColor.image,
		       VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
		       customStuff.reachableImage.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &ic);
		       

	cmdSetLayout(customStuff.secondCommandBuffer, customStuff.fbColor.image, VK_IMAGE_ASPECT_COLOR_BIT,
		     VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
	cmdSetLayout(customStuff.secondCommandBuffer, customStuff.reachableImage.image, VK_IMAGE_ASPECT_COLOR_BIT,
		     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL);

	VK_CHECK_RESULT(vkEndCommandBuffer(customStuff.secondCommandBuffer));


	VkSubmitInfo submitInfo = {};
	VkPipelineStageFlags waitDstStageMask2 = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
	submitInfo.pNext = NULL;
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	// submitInfo.waitSemaphoreCount = 0;
	// submitInfo.pWaitSemaphores = NULL;
	// submitInfo.pWaitDstStageMask = NULL;
	submitInfo.waitSemaphoreCount = 1;
	submitInfo.pWaitSemaphores = &customStuff.renderedSemaphore;
	submitInfo.pWaitDstStageMask = &waitDstStageMask2;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &customStuff.secondCommandBuffer;

	submitInfo.signalSemaphoreCount = 0;
	// submitInfo.pSignalSemaphores = NULL;
	// submitInfo.signalSemaphoreCount = 1;
	// submitInfo.pSignalSemaphores = &customStuff.copiedSemaphore;
	
	// Wait until rendering is done

	VkResult res;
	do{
	    res = vkWaitForFences(device, 1, &customStuff.fence, VK_TRUE, 10000000);
	} while (res == VK_TIMEOUT);
	
	VK_CHECK_RESULT(vkResetFences(device, 1, &customStuff.fence));

	VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, customStuff.fence));
	
	// Wait until image copy is done
	
	do{
	    res = vkWaitForFences(device, 1, &customStuff.fence, VK_TRUE, 10000);
	} while (res == VK_TIMEOUT);
	
	VK_CHECK_RESULT(vkResetFences(device, 1, &customStuff.fence));

	VkImageSubresource subres{};
	subres.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	subres.mipLevel = 0;
	subres.arrayLayer = 0;
	VkSubresourceLayout srl;
	
	vkGetImageSubresourceLayout(device, customStuff.reachableImage.image, &subres, &srl);


	// Reset camera
	camera.setPerspective(45.0, (float)width / (float)height, 0.001f, 256.0f);
	updateUniformBuffers();
	

	
	using out_type = float;
        out_type* tmp;
	
	VK_CHECK_RESULT(vkMapMemory(device, customStuff.reachableImage.memory, 0, srl.size, 0, (void**)&tmp));

	
	tmp += srl.offset / sizeof(out_type);

        out_type* data = new out_type[this->height * this->width * 4];
	// Reverse byte order
	if( srl.rowPitch == this->width * sizeof(out_type) * 4) {
	    memcpy(data, tmp, srl.size);
	} else {
	    float* dataP = data;
	    for(uint32_t i = 0; i < this->height; i++) {

		out_type *tp = (out_type*)tmp;
		for(uint32_t j = 0; j < this->width; j++) {
		    for(int k = 0; k < 4; k++) {
			*(dataP++) = *(tp++);
		    }
		}
	    
		tmp += srl.rowPitch / sizeof(out_type);
	    
	    }
	}

	vkUnmapMemory(device, customStuff.reachableImage.memory);
	
	std::ostringstream oss;
	oss << settings.output_prefixes[feature_index]  << std::setfill('0') << std::setw(OUTPUT_INDEX_PAD) << count << ".exr";
	std::string filename = oss.str();

	// Destructively convert to 3-channel image
	to3chan(data, this->width, this->height);
	output_image_float(data, this->width, this->height, 3, filename);
	
	delete[] data;

	count++;
	std::cout << "Image saved to " << filename << std::endl;
    }

    void destroyCustomStuff() {
	vkDestroyFence(device, customStuff.fence, nullptr);
	vkDestroySemaphore(device, customStuff.renderedSemaphore, nullptr);
	vkDestroySemaphore(device, customStuff.copiedSemaphore, nullptr);

	vkDestroyImage(device, customStuff.reachableImage.image, nullptr);
	vkFreeMemory(device, customStuff.reachableImage.memory, nullptr);

	vkDestroyImageView(device, customStuff.fbColor.view, nullptr);
	vkDestroyImage(device, customStuff.fbColor.image, nullptr);
	vkFreeMemory(device, customStuff.fbColor.memory, nullptr);

	vkDestroyImageView(device, customStuff.fbDepth.view, nullptr);
	vkDestroyImage(device, customStuff.fbDepth.image, nullptr);
	vkFreeMemory(device, customStuff.fbDepth.memory, nullptr);

	vkDestroyRenderPass(device, customStuff.renderPass, nullptr);
	
	vkDestroyFramebuffer(device, customStuff.framebuffer, nullptr);
    }

    // Function for setting up screenshot-related stuff
    void setupCustomStuff() {

      std::cout << "Starting custom setup" << std::endl;
      
	// Create RenderPass
	VkAttachmentDescription atts[2] = {};
	atts[0].format = CUSTOM_FORMAT; // swapChain.colorFormat;
	atts[0].samples = VK_SAMPLE_COUNT_1_BIT;
	atts[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	atts[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	atts[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	atts[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	atts[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	atts[0].finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	atts[1].format = depthFormat;
	atts[1].samples = VK_SAMPLE_COUNT_1_BIT;
	atts[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	atts[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	atts[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	atts[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	atts[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	atts[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	VkAttachmentReference cr = {};
	cr.attachment = 0;
	cr.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	VkAttachmentReference dr = {};
	dr.attachment = 1;
	dr.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	VkSubpassDescription sd = {};
	sd.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	sd.colorAttachmentCount = 1;
	sd.pColorAttachments = &cr;
	sd.pDepthStencilAttachment = &dr;
	sd.inputAttachmentCount = 0;
	sd.pInputAttachments = nullptr;
	sd.preserveAttachmentCount = 0;
	sd.pPreserveAttachments = nullptr;
	sd.pResolveAttachments = nullptr;

	VkSubpassDependency deps[2];
	deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
	deps[0].dstSubpass = 0;
	deps[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
	deps[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	deps[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
	deps[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	deps[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

	deps[1].srcSubpass = 0;
	deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
	deps[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	deps[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
	deps[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	deps[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
	deps[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

	VkRenderPassCreateInfo rpci{};
	rpci.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	rpci.attachmentCount = static_cast<uint32_t>(2);
	rpci.pAttachments = atts;
	rpci.subpassCount = 1;
	rpci.pSubpasses = &sd;
	rpci.dependencyCount = static_cast<uint32_t>(2);
	rpci.pDependencies = deps;
	VK_CHECK_RESULT(vkCreateRenderPass(device, &rpci, nullptr, &customStuff.renderPass));

	// Create fence
	VkFenceCreateInfo fci{};
	fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fci.pNext = nullptr;
	fci.flags = 0;

	VK_CHECK_RESULT(vkCreateFence(device, &fci, nullptr, &customStuff.fence));

	// Create semaphores
	VkSemaphoreCreateInfo sci{};
	sci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	sci.pNext = nullptr;
	sci.flags = 0;

	VK_CHECK_RESULT(vkCreateSemaphore(device, &sci, nullptr, &customStuff.renderedSemaphore));
	VK_CHECK_RESULT(vkCreateSemaphore(device, &sci, nullptr, &customStuff.copiedSemaphore));

	// Create host-reachable image (with memory)
	VkImageCreateInfo ici{};
	ici.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	ici.imageType = VK_IMAGE_TYPE_2D;
	ici.format = CUSTOM_FORMAT; // swapChain.colorFormat;
	ici.extent.width = this->width;
	ici.extent.height = this->height;
	ici.extent.depth = 1;
	ici.mipLevels = 1;
	ici.arrayLayers = 1;
	ici.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	ici.tiling = VK_IMAGE_TILING_LINEAR;
	ici.samples = VK_SAMPLE_COUNT_1_BIT;
	ici.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	VK_CHECK_RESULT(vkCreateImage(device, &ici, nullptr, &customStuff.reachableImage.image));

	VkMemoryRequirements rMemReqs;
	vkGetImageMemoryRequirements(device, customStuff.reachableImage.image, &rMemReqs);
	VkMemoryAllocateInfo rMemAllocInfo{};
	rMemAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	rMemAllocInfo.allocationSize = rMemReqs.size;
	VkBool32 rLazyMemTypePresent;
	rMemAllocInfo.memoryTypeIndex = vulkanDevice->getMemoryType(rMemReqs.memoryTypeBits, VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT | VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, &rLazyMemTypePresent);
	if (!rLazyMemTypePresent) {
	    rMemAllocInfo.memoryTypeIndex = vulkanDevice->getMemoryType(rMemReqs.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
	}
	VK_CHECK_RESULT(vkAllocateMemory(device, &rMemAllocInfo, nullptr, &customStuff.reachableImage.memory));
	vkBindImageMemory(device, customStuff.reachableImage.image, customStuff.reachableImage.memory, 0);

	customStuff.reachableImage.memorySize = rMemReqs.size;


	
	// Mainly copied from the setupFrameBuffer function
	// Create framebuffer with depth and color images
	VkImageCreateInfo imageCI{};
	imageCI.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	imageCI.imageType = VK_IMAGE_TYPE_2D;
	imageCI.format = CUSTOM_FORMAT; // swapChain.colorFormat;
	imageCI.extent.width = this->width;
	imageCI.extent.height = this->height;
	imageCI.extent.depth = 1;
	imageCI.mipLevels = 1;
	imageCI.arrayLayers = 1;
	imageCI.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	imageCI.tiling = VK_IMAGE_TILING_OPTIMAL;
	imageCI.samples = VK_SAMPLE_COUNT_1_BIT;
	imageCI.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
	    VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
	imageCI.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	VK_CHECK_RESULT(vkCreateImage(device, &imageCI, nullptr, &customStuff.fbColor.image));

	VkMemoryRequirements memReqs;
	vkGetImageMemoryRequirements(device, customStuff.fbColor.image, &memReqs);
	VkMemoryAllocateInfo memAllocInfo{};
	memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	memAllocInfo.allocationSize = memReqs.size;
	memAllocInfo.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	VK_CHECK_RESULT(vkAllocateMemory(device, &memAllocInfo, nullptr, &customStuff.fbColor.memory));
	vkBindImageMemory(device, customStuff.fbColor.image, customStuff.fbColor.memory, 0);

	// Create image view for the MSAA target
	VkImageViewCreateInfo imageViewCI{};
	imageViewCI.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	imageViewCI.image = customStuff.fbColor.image;
	imageViewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
	imageViewCI.format = CUSTOM_FORMAT; // swapChain.colorFormat;
	imageViewCI.components.r = VK_COMPONENT_SWIZZLE_R;
	imageViewCI.components.g = VK_COMPONENT_SWIZZLE_G;
	imageViewCI.components.b = VK_COMPONENT_SWIZZLE_B;
	imageViewCI.components.a = VK_COMPONENT_SWIZZLE_A;
	imageViewCI.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	imageViewCI.subresourceRange.levelCount = 1;
	imageViewCI.subresourceRange.layerCount = 1;
	VK_CHECK_RESULT(vkCreateImageView(device, &imageViewCI, nullptr, &customStuff.fbColor.view));

	// Depth target
	imageCI.imageType = VK_IMAGE_TYPE_2D;
	imageCI.format = depthFormat;
	imageCI.extent.width = this->width;
	imageCI.extent.height = this->height;
	imageCI.extent.depth = 1;
	imageCI.mipLevels = 1;
	imageCI.arrayLayers = 1;
	imageCI.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	imageCI.tiling = VK_IMAGE_TILING_OPTIMAL;
	imageCI.samples = settings.sampleCount;
	imageCI.usage = VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
	imageCI.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	VK_CHECK_RESULT(vkCreateImage(device, &imageCI, nullptr, &customStuff.fbDepth.image));

	vkGetImageMemoryRequirements(device, customStuff.fbDepth.image, &memReqs);
	memAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	memAllocInfo.allocationSize = memReqs.size;
	VkBool32 lazyMemTypePresent;
	memAllocInfo.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT, &lazyMemTypePresent);
	if (!lazyMemTypePresent) {
	    memAllocInfo.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
	}
	VK_CHECK_RESULT(vkAllocateMemory(device, &memAllocInfo, nullptr, &customStuff.fbDepth.memory));
	vkBindImageMemory(device, customStuff.fbDepth.image, customStuff.fbDepth.memory, 0);

	// Create image view for the MSAA target
	imageViewCI.image = customStuff.fbDepth.image;
	imageViewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
	imageViewCI.format = depthFormat;
	imageViewCI.components.r = VK_COMPONENT_SWIZZLE_R;
	imageViewCI.components.g = VK_COMPONENT_SWIZZLE_G;
	imageViewCI.components.b = VK_COMPONENT_SWIZZLE_B;
	imageViewCI.components.a = VK_COMPONENT_SWIZZLE_A;
	imageViewCI.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
	imageViewCI.subresourceRange.levelCount = 1;
	imageViewCI.subresourceRange.layerCount = 1;
	VK_CHECK_RESULT(vkCreateImageView(device, &imageViewCI, nullptr, &customStuff.fbDepth.view));
	
	VkImageView attachments[2];

	attachments[0] = customStuff.fbColor.view;
	attachments[1] = customStuff.fbDepth.view;
	
	VkFramebufferCreateInfo fbci{};
	fbci.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
	fbci.pNext = NULL;
	fbci.renderPass = customStuff.renderPass;
	fbci.attachmentCount = 2;
	fbci.pAttachments = attachments;
	fbci.width = this->width;
	fbci.height = this->height;
	fbci.layers = 1;

	VK_CHECK_RESULT(vkCreateFramebuffer(device, &fbci, nullptr, &customStuff.framebuffer));

	VkCommandBufferAllocateInfo cbai;
	cbai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	cbai.pNext = nullptr;
	cbai.commandPool = cmdPool;
	cbai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	cbai.commandBufferCount = commandBuffers.size();

	VK_CHECK_RESULT(vkAllocateCommandBuffers(device, &cbai, customStuff.commandBuffers));

	// customStuff.commandBuffers = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, false);
	customStuff.secondCommandBuffer = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, false);

	
	std::cout << "Completed custom setup" << std::endl;
    } 

#ifdef WITH_DISPLAY
	/*
		Update ImGui user interface
	*/
	void updateOverlay()
	{
		ImGuiIO& io = ImGui::GetIO();

		ImVec2 lastDisplaySize = io.DisplaySize;
		io.DisplaySize = ImVec2((float)width, (float)height);
		io.DeltaTime = frameTimer;

		io.MousePos = ImVec2(mousePos.x, mousePos.y);
		io.MouseDown[0] = mouseButtons.left;
		io.MouseDown[1] = mouseButtons.right;

		ui->pushConstBlock.scale = glm::vec2(2.0f / io.DisplaySize.x, 2.0f / io.DisplaySize.y);
		ui->pushConstBlock.translate = glm::vec2(-1.0f);

		bool updateShaderParams = false;
		bool updateCBs = false;
		float scale = 1.0f;

#if defined(VK_USE_PLATFORM_ANDROID_KHR)
		scale = (float)vks::android::screenDensity / (float)ACONFIGURATION_DENSITY_MEDIUM;
#endif
		ImGui::NewFrame();

		ImGui::SetNextWindowPos(ImVec2(10, 10));
		ImGui::SetNextWindowSize(ImVec2(200 * scale, (models.scene.animations.size() > 0 ? 440 : 360) * scale), ImGuiSetCond_Always);
		ImGui::Begin("Vulkan glTF 2.0 PBR", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
		ImGui::PushItemWidth(100.0f * scale);

		ui->text("www.saschawillems.de");
		ui->text("%.1d fps (%.2f ms)", lastFPS, (1000.0f / lastFPS));

		if (ui->header("Scene")) {
#if defined(VK_USE_PLATFORM_ANDROID_KHR)
			if (ui->combo("File", selectedScene, scenes)) {
				vkDeviceWaitIdle(device);
				loadScene(scenes[selectedScene]);
				setupDescriptors();
				updateCBs = true;
			}
#else
			if (ui->button("Open gltf file")) {
				std::string filename = "";
#if defined(_WIN32)
				char buffer[MAX_PATH];
				OPENFILENAME ofn;
				ZeroMemory(&buffer, sizeof(buffer));
				ZeroMemory(&ofn, sizeof(ofn));
				ofn.lStructSize = sizeof(ofn);
				ofn.lpstrFilter = "glTF files\0*.gltf;*.glb\0";
				ofn.lpstrFile = buffer;
				ofn.nMaxFile = MAX_PATH;
				ofn.lpstrTitle = "Select a glTF file to load";
				ofn.Flags = OFN_DONTADDTORECENT | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
				if (GetOpenFileNameA(&ofn)) {
					filename = buffer;
				}
#elif defined(__linux__) && !defined(VK_USE_PLATFORM_ANDROID_KHR)
				char buffer[1024];
				FILE *file = popen("zenity --title=\"Select a glTF file to load\" --file-filter=\"glTF files | *.gltf *.glb\" --file-selection", "r");
				if (file) {
					while (fgets(buffer, sizeof(buffer), file)) {
						filename += buffer;
					};
					filename.erase(std::remove(filename.begin(), filename.end(), '\n'), filename.end());
					std::cout << filename << std::endl;
				}
#endif
				if (!filename.empty()) {
					vkDeviceWaitIdle(device);
					loadScene(filename);
					setupDescriptors();
					updateCBs = true;
				}
			}
#endif
			if (ui->combo("Environment", selectedEnvironment, environments)) {
				vkDeviceWaitIdle(device);
				loadEnvironment(environments[selectedEnvironment]);
				setupDescriptors();
				updateCBs = true;
			}
		}

		if (ui->header("Environment")) {
			if (ui->checkbox("Background", &displayBackground)) {
				updateShaderParams = true;
			}
			if (ui->slider("Exposure", &shaderValuesParams.exposure, 0.1f, 10.0f)) {
				updateShaderParams = true;
			}
			if (ui->slider("Gamma", &shaderValuesParams.gamma, 0.1f, 4.0f)) {
				updateShaderParams = true;
			}
			if (ui->slider("IBL", &shaderValuesParams.scaleIBLAmbient, 0.0f, 1.0f)) {
				updateShaderParams = true;
			}
		}

		if (ui->header("Debug view")) {
			const std::vector<std::string> debugNamesInputs = {
				"none", "Base color", "Normal", "Occlusion", "Emissive", "Metallic", "Roughness"
			};
			if (ui->combo("Inputs", &debugViewInputs, debugNamesInputs)) {
				shaderValuesParams.debugViewInputs = debugViewInputs;
				updateShaderParams = true;
			}
			const std::vector<std::string> debugNamesEquation = {
				"none", "Diff (l,n)", "F (l,h)", "G (l,v,h)", "D (h)", "Specular"
			};
			if (ui->combo("PBR equation", &debugViewEquation, debugNamesEquation)) {
				shaderValuesParams.debugViewEquation = debugViewEquation;
				updateShaderParams = true;
			}
		}

		if (models.scene.animations.size() > 0) {
			if (ui->header("Animations")) {
				ui->checkbox("Animate", &animate);
				std::vector<std::string> animationNames;
				for (auto animation : models.scene.animations) {
					animationNames.push_back(animation.name);
				}
				ui->combo("Animation", &animationIndex, animationNames);
			}
		}

		ImGui::PopItemWidth();
		ImGui::End();
		ImGui::Render();

		ImDrawData* imDrawData = ImGui::GetDrawData();

		// Check if ui buffers need to be recreated
		if (imDrawData) {
			VkDeviceSize vertexBufferSize = imDrawData->TotalVtxCount * sizeof(ImDrawVert);
			VkDeviceSize indexBufferSize = imDrawData->TotalIdxCount * sizeof(ImDrawIdx);

			bool updateBuffers = (ui->vertexBuffer.buffer == VK_NULL_HANDLE) || (ui->vertexBuffer.count != imDrawData->TotalVtxCount) || (ui->indexBuffer.buffer == VK_NULL_HANDLE) || (ui->indexBuffer.count != imDrawData->TotalIdxCount);

			if (updateBuffers) {
				vkDeviceWaitIdle(device);
				if (ui->vertexBuffer.buffer) {
					ui->vertexBuffer.destroy();
				}
				ui->vertexBuffer.create(vulkanDevice, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, vertexBufferSize);
				ui->vertexBuffer.count = imDrawData->TotalVtxCount;
				if (ui->indexBuffer.buffer) {
					ui->indexBuffer.destroy();
				}
				ui->indexBuffer.create(vulkanDevice, VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT, indexBufferSize);
				ui->indexBuffer.count = imDrawData->TotalIdxCount;
			}

			// Upload data
			ImDrawVert* vtxDst = (ImDrawVert*)ui->vertexBuffer.mapped;
			ImDrawIdx* idxDst = (ImDrawIdx*)ui->indexBuffer.mapped;
			for (int n = 0; n < imDrawData->CmdListsCount; n++) {
				const ImDrawList* cmd_list = imDrawData->CmdLists[n];
				memcpy(vtxDst, cmd_list->VtxBuffer.Data, cmd_list->VtxBuffer.Size * sizeof(ImDrawVert));
				memcpy(idxDst, cmd_list->IdxBuffer.Data, cmd_list->IdxBuffer.Size * sizeof(ImDrawIdx));
				vtxDst += cmd_list->VtxBuffer.Size;
				idxDst += cmd_list->IdxBuffer.Size;
			}

			ui->vertexBuffer.flush();
			ui->indexBuffer.flush();

			updateCBs = updateCBs || updateBuffers;
		}

		if (lastDisplaySize.x != io.DisplaySize.x || lastDisplaySize.y != io.DisplaySize.y) {
			updateCBs = true;
		}

		if (updateCBs) {
			vkDeviceWaitIdle(device);
			recordCommandBuffers();
			vkDeviceWaitIdle(device);
		}

		if (updateShaderParams) {
			updateParams();
		}

#if defined(VK_USE_PLATFORM_ANDROID_KHR)
		if (mouseButtons.left) {
			mouseButtons.left = false;
		}
#endif
	}

#endif // WITH_DISPLAY

	virtual void render()
  {
		if (!prepared) {
			return;
		}

		// The BMFR counting starts from 1
		const size_t start_count = settings.interval_t0 == -1 ? 0 : settings.interval_t0;
		const size_t end_count = settings.interval_t1 == -1 ? settings.pathViews.size() : settings.interval_t1 + 1;
				
		static size_t count = start_count;
		static size_t feature_count = 0;
		
		if(settings.followPath) {
		  if(count >= end_count) {
		    if(settings.feature_buffers.size()) {
		      std::cout << "Done with " << settings.feature_buffers[feature_count] << std::endl;
		      count = start_count;
		      feature_count++;
		      if(feature_count >= settings.feature_buffers.size()) {
			std::cout << "Done following path, exiting" << std::endl;
			this->quit = true;
			return;
		      }
		    } else {
		      std::cout << "Done following path, exiting" << std::endl;
		    }
		      
		  }
		  
		  std::pair<glm::vec3, glm::vec3> decomp = settings.pathViews[count];
		  camera.setRotation(decomp.first);
		  camera.setPosition(decomp.second);
		}
		

		if(count == start_count && settings.feature_buffers.size()) {
		  bool ok = false;
		  for(int i = 0; i < num_available_features; i++) {
		    if(available_features[i] == settings.feature_buffers[feature_count]) {
		      shaderValuesParams.debugViewEquation = i;
		      ok = true;
		      break;
		    }
		  }
		  if (!ok) {
		    std::cout << "Debug value not set!" << std::endl;
		    std::cout << "feature name: " << settings.feature_buffers[feature_count] << std::endl;
		  }
		}

		// Update UBOs
		updateUniformBuffers();
		UniformBufferSet currentUB = uniformBuffers[currentBuffer];
		memcpy(currentUB.scene.mapped, &shaderValuesScene, sizeof(shaderValuesScene));
		memcpy(currentUB.params.mapped, &shaderValuesParams, sizeof(shaderValuesParams));
		memcpy(currentUB.skybox.mapped, &shaderValuesSkybox, sizeof(shaderValuesSkybox));
		
		
		renderCustom(count + settings.start_index, feature_count);
		count++;
		
		if (camera.updated) {
			updateUniformBuffers();
		}
	}
};

VulkanExample *vulkanExample;

// OS specific macros for the example main entry points
#if defined(_WIN32)
LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	if (vulkanExample != NULL)
	{
		vulkanExample->handleMessages(hWnd, uMsg, wParam, lParam);
	}
	return (DefWindowProc(hWnd, uMsg, wParam, lParam));
}
int APIENTRY WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int)
{
	for (int32_t i = 0; i < __argc; i++) { VulkanExample::args.push_back(__argv[i]); };
	vulkanExample = new VulkanExample();
	vulkanExample->initVulkan();
	vulkanExample->setupWindow(hInstance, WndProc);
	vulkanExample->prepare();
	vulkanExample->renderLoop();
	delete(vulkanExample);
	return 0;
}
#elif defined(VK_USE_PLATFORM_ANDROID_KHR)
// Android entry point
void android_main(android_app* state)
{
	vulkanExample = new VulkanExample();
	state->userData = vulkanExample;
	state->onAppCmd = VulkanExample::handleAppCommand;
	state->onInputEvent = VulkanExample::handleAppInput;
	androidApp = state;
	vks::android::getDeviceConfig();
	vulkanExample->renderLoop();
	delete(vulkanExample);
}
#elif defined(_DIRECT2DISPLAY)
// Linux entry point with direct to display wsi
static void handleEvent()
{
}
int main(const int argc, const char *argv[])
{
	for (size_t i = 0; i < argc; i++) { VulkanExample::args.push_back(argv[i]); };
	vulkanExample = new VulkanExample();
	vulkanExample->initVulkan();
	vulkanExample->prepare();
	vulkanExample->renderLoop();
	delete(vulkanExample);
	return 0;
}
#elif defined(VK_USE_PLATFORM_WAYLAND_KHR)
int main(const int argc, const char *argv[])
{
	for (size_t i = 0; i < argc; i++) { VulkanExample::args.push_back(argv[i]); };
	vulkanExample = new VulkanExample();
	vulkanExample->initVulkan();
	vulkanExample->setupWindow();
	vulkanExample->prepare();
	vulkanExample->renderLoop();
	delete(vulkanExample);
	return 0;
}
#elif defined(VK_USE_PLATFORM_XCB_KHR)

int main(const int argc, const char *argv[])
{
	for (int i = 0; i < argc; i++) { VulkanExample::args.push_back(argv[i]); };
	vulkanExample = new VulkanExample();
	vulkanExample->initVulkan();
#ifdef WITH_DISPLAY
	vulkanExample->setupWindow();
#endif // WITH_DISPLAY
	vulkanExample->prepare();
	vulkanExample->renderLoop();
	delete(vulkanExample);
	return 0;
}
#endif
