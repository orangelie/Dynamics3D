#pragma once

#if defined(ZEKROS_ENGINE)
#undef ZEKROS_ENGINE
#endif
#define ZEKROS_ENGINE orangelie::RigidBodySim

#include "Renderer.h"

namespace orangelie
{
	using namespace FrameResources::RigidBodySim;

	class RigidBodySim : public Renderer
	{
	private:
		bool isSimulating = true;
		const real mhSize = 1.0f;
		Vector3 mHalfSize = { mhSize, mhSize, mhSize };
		RigidBody mRigidBody1, mRigidBody2, mRigidBody3;
		Contact mContact[256] = {};
		std::unique_ptr<ContactResolver> mContactResolver;
		CollisionBox mCBox1, mCBox2, mCBox3;
		CollisionData mCData;


		std::unordered_map<std::string, std::vector<Shader::FontType>> mFontData;
		const static int gMaxNumTextCharacters = 256;	//< Must be greater than 256 and must be a multiple of 16 >//
		Shader::RenderItem* mTextVB = nullptr;
		Shader::RenderItem* mBox1VB = nullptr;
		Shader::RenderItem* mBox2VB = nullptr;
		Shader::RenderItem* mBox3VB = nullptr;

		Camera mCamera;
		std::vector<std::unique_ptr<FrameResource>> mFrameResources;
		FrameResource* mCurrFrameResource = nullptr;
		UINT mCurrFrameResourceIndex = 0;

		std::unordered_map<std::string, std::unique_ptr<Shader::Texture>> mTextures;
		std::unordered_map<std::string, Microsoft::WRL::ComPtr<ID3DBlob>> mShaders;
		std::unordered_map<std::string, std::unique_ptr<Shader::Material>> mMaterials;
		std::unordered_map<std::string, std::vector<D3D12_INPUT_ELEMENT_DESC>> mInputLayouts;
		std::unordered_map<std::string, std::unique_ptr<Shader::MeshGeometry>> mDrawArgs;
		std::vector<std::unique_ptr<Shader::RenderItem>> mAllRenderItems;
		std::vector<Shader::RenderItem*> mRenderLayer[(size_t)Shader::RenderLayer::Count];
		Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> mSrvHeap = nullptr;
		Microsoft::WRL::ComPtr<ID3D12RootSignature> mRootSignatrue = nullptr;
		std::unordered_map<std::string, Microsoft::WRL::ComPtr<ID3D12PipelineState>> mGraphicsPSO;

		INT mLastMousePosX, mLastMousePosY;

		void BuildTexture()
		{
			auto textTexture = std::make_unique<Shader::Texture>();
			textTexture->name = "text";
			textTexture->fileName = L"text.png";

			HR(WICConverter::CreateWICTextureFromFile12(mDevice.Get(),
				mGraphicsCommandList.Get(),
				textTexture->fileName.c_str(),
				textTexture->ResourceGpuHeap,
				textTexture->UploadGpuHeap));

			mTextures[textTexture->name] = std::move(textTexture);

			auto woodTexture = std::make_unique<Shader::Texture>();
			woodTexture->name = "wood";
			woodTexture->fileName = L"wood.png";

			HR(WICConverter::CreateWICTextureFromFile12(mDevice.Get(),
				mGraphicsCommandList.Get(),
				woodTexture->fileName.c_str(),
				woodTexture->ResourceGpuHeap,
				woodTexture->UploadGpuHeap));

			mTextures[woodTexture->name] = std::move(woodTexture);

			/*
			HR(DirectX::CreateDDSTextureFromFile12(mDevice.Get(),
				mGraphicsCommandList.Get(),
				tempTexture->fileName.c_str(),
				tempTexture->ResourceGpuHeap,
				tempTexture->UploadGpuHeap));
			*/
		}

		void BuildRootSignature()
		{
			CD3DX12_DESCRIPTOR_RANGE srvRange;
			srvRange.Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 2, 0, 0);

			constexpr const size_t parameterSize = 3;
			CD3DX12_ROOT_PARAMETER rootParameter[parameterSize];
			rootParameter[0].InitAsConstantBufferView(0);												// Object
			rootParameter[1].InitAsConstantBufferView(1);												// Pass
			rootParameter[2].InitAsDescriptorTable(1, &srvRange, D3D12_SHADER_VISIBILITY_PIXEL);		// Textures

			auto staticSamplers = GetStaticSamplers();

			CD3DX12_ROOT_SIGNATURE_DESC rootSignatureDescriptor = {};
			rootSignatureDescriptor.Init(parameterSize, rootParameter, (UINT)staticSamplers.size(), staticSamplers.data(),
				D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

			Microsoft::WRL::ComPtr<ID3DBlob> ppBlob, ppErrorMsg;
			HR(D3D12SerializeRootSignature(&rootSignatureDescriptor,
				D3D_ROOT_SIGNATURE_VERSION_1, ppBlob.GetAddressOf(), ppErrorMsg.GetAddressOf()));

			if (ppErrorMsg != nullptr)
			{
				MessageBoxA(0, (char*)ppErrorMsg->GetBufferPointer(), "RootSignature Error", MB_OK);
				throw std::runtime_error("RootSignature Throws");
			}

			HR(mDevice->CreateRootSignature(0,
				ppBlob->GetBufferPointer(), ppBlob->GetBufferSize(), IID_PPV_ARGS(mRootSignatrue.GetAddressOf())));
		}

		void BuildDescriptor()
		{
			D3D12_DESCRIPTOR_HEAP_DESC srvHeapDescriptor = {};
			srvHeapDescriptor.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
			srvHeapDescriptor.NodeMask = 0;
			srvHeapDescriptor.NumDescriptors = 2;
			srvHeapDescriptor.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;

			HR(mDevice->CreateDescriptorHeap(&srvHeapDescriptor, IID_PPV_ARGS(mSrvHeap.GetAddressOf())));
			CD3DX12_CPU_DESCRIPTOR_HANDLE srvHandle(mSrvHeap->GetCPUDescriptorHandleForHeapStart());

			D3D12_SHADER_RESOURCE_VIEW_DESC srViewDescriptor = {};
			srViewDescriptor.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
			srViewDescriptor.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
			srViewDescriptor.Texture2D.MostDetailedMip = 0;
			srViewDescriptor.Texture2D.ResourceMinLODClamp = 0.0f;

			auto textTexture = mTextures["text"].get()->ResourceGpuHeap;
			srViewDescriptor.Format = textTexture->GetDesc().Format;
			srViewDescriptor.Texture2D.MipLevels = textTexture->GetDesc().MipLevels;

			mDevice->CreateShaderResourceView(textTexture.Get(), &srViewDescriptor, srvHandle);
			srvHandle.Offset(1, mCbvSrvUavSize);


			auto woodTexture = mTextures["wood"].get()->ResourceGpuHeap;
			srViewDescriptor.Format = woodTexture->GetDesc().Format;
			srViewDescriptor.Texture2D.MipLevels = woodTexture->GetDesc().MipLevels;

			mDevice->CreateShaderResourceView(woodTexture.Get(), &srViewDescriptor, srvHandle);
			srvHandle.Offset(1, mCbvSrvUavSize);
		}

		void BuildShadersAndInputLayout()
		{
			D3D_SHADER_MACRO shaderMacro[] =
			{
				"SHADER", "0",
				NULL, NULL
			};

			mShaders["vs"] = Utils::CompileShader(L"OpaqueShader.hlsl", shaderMacro, "VS", "vs_5_1");
			mShaders["ps"] = Utils::CompileShader(L"OpaqueShader.hlsl", shaderMacro, "PS", "ps_5_1");
			mShaders["text_vs"] = Utils::CompileShader(L"TextShader.hlsl", shaderMacro, "VS", "vs_5_1");
			mShaders["text_ps"] = Utils::CompileShader(L"TextShader.hlsl", shaderMacro, "PS", "ps_5_1");

			mInputLayouts["layout1"] = {
				{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
				{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
				{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
				{ "TANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 32, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 }
			};

			mInputLayouts["layout2"] = {
				{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
				{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 }
			};
		}

		void BuildMaterial()
		{
			auto text = std::make_unique<Shader::Material>();
			text->SrvTextureIndex = 0;

			mMaterials["text"] = std::move(text);

			auto wood = std::make_unique<Shader::Material>();
			wood->SrvTextureIndex = 1;

			mMaterials["wood"] = std::move(wood);
		}

		void BuildShapeMesh()
		{
			GeometryGenerator geoGen;
			GeometryGenerator::MeshData box = geoGen.CreateBox(2.0f, 2.0f, 2.0f, 4);
			GeometryGenerator::MeshData sphere = geoGen.CreateSphere(1.0f, 20, 20);

			UINT k = 0;
			std::vector<Shader::Vertex> vertices(box.Vertices.size() + sphere.Vertices.size());

			for (size_t i = 0; i < box.Vertices.size(); ++i, ++k)
			{
				vertices[k].Position = box.Vertices[i].Position;
				vertices[k].Normal = box.Vertices[i].Normal;
				vertices[k].Tangent = box.Vertices[i].Tangent;
				vertices[k].TexCoord = box.Vertices[i].TexCoord;
			}
			for (size_t i = 0; i < sphere.Vertices.size(); ++i, ++k)
			{
				vertices[k].Position = sphere.Vertices[i].Position;
				vertices[k].Normal = sphere.Vertices[i].Normal;
				vertices[k].Tangent = sphere.Vertices[i].Tangent;
				vertices[k].TexCoord = sphere.Vertices[i].TexCoord;
			}

			std::vector<std::uint32_t> indices = box.Indices;
			indices.insert(indices.end(), sphere.Indices.begin(), sphere.Indices.end());

			UINT vertexBufferSize = sizeof(Shader::Vertex) * (UINT)vertices.size();
			UINT indexBufferSize = sizeof(std::uint32_t) * (UINT)indices.size();

			auto meshGeometry = std::make_unique<Shader::MeshGeometry>();
			meshGeometry->name = "shapeGeo";

			HR(D3DCreateBlob(vertexBufferSize, meshGeometry->VertexCpu.GetAddressOf()));
			CopyMemory(meshGeometry->VertexCpu->GetBufferPointer(), vertices.data(), vertexBufferSize);
			HR(D3DCreateBlob(indexBufferSize, meshGeometry->IndexCpu.GetAddressOf()));
			CopyMemory(meshGeometry->IndexCpu->GetBufferPointer(), indices.data(), indexBufferSize);

			meshGeometry->VertexGpu = Utils::CreateDefaultResource(mDevice.Get(), mGraphicsCommandList.Get(),
				vertices.data(), vertexBufferSize, meshGeometry->VertexGpuUploader);
			meshGeometry->IndexGpu = Utils::CreateDefaultResource(mDevice.Get(), mGraphicsCommandList.Get(),
				indices.data(), indexBufferSize, meshGeometry->IndexGpuUploader);

			meshGeometry->VertexBufferByteSize = vertexBufferSize;
			meshGeometry->VertexByteStride = sizeof(Shader::Vertex);

			meshGeometry->IndexFormat = DXGI_FORMAT_R32_UINT;
			meshGeometry->IndexBufferByteSize = indexBufferSize;

			Shader::SubmeshGeometry subMeshBox = {};
			subMeshBox.IndexCount = (UINT)box.Indices.size();
			subMeshBox.BaseVertexLocation = 0;
			subMeshBox.StartIndexLocation = 0;
			meshGeometry->DrawArgs["box"] = subMeshBox;

			Shader::SubmeshGeometry subMeshSphere = {};
			subMeshSphere.IndexCount = (UINT)sphere.Indices.size();
			subMeshSphere.BaseVertexLocation = (INT)box.Vertices.size();
			subMeshSphere.StartIndexLocation = (UINT)box.Indices.size();
			meshGeometry->DrawArgs["sphere"] = subMeshSphere;

			mDrawArgs[meshGeometry->name] = std::move(meshGeometry);
		}

		void BuildFont()
		{
			mFontData["original"] = TextFont::LoadFontData("fontdata.txt");
			
			std::vector<Shader::TextVertex> vertices(gMaxNumTextCharacters * 4);
			std::vector<std::uint32_t> indices(gMaxNumTextCharacters * 6);

			for (size_t i = 0, k = 0; i < indices.size(); i += 6, k += 4)
			{
				// 0 1
				// 2 3

				indices[i + 0] = (u32)k + 0;
				indices[i + 1] = (u32)k + 1;
				indices[i + 2] = (u32)k + 2;

				indices[i + 3] = (u32)k + 1;
				indices[i + 4] = (u32)k + 3;
				indices[i + 5] = (u32)k + 2;
			}

			UINT vertexBufferSize = sizeof(Shader::TextVertex) * (UINT)vertices.size();
			UINT indexBufferSize = sizeof(std::uint32_t) * (UINT)indices.size();

			auto meshGeometry = std::make_unique<Shader::MeshGeometry>();
			meshGeometry->name = "textGeo";

			meshGeometry->VertexCpu = nullptr;
			meshGeometry->VertexGpu = nullptr;

			HR(D3DCreateBlob(indexBufferSize, meshGeometry->IndexCpu.GetAddressOf()));
			CopyMemory(meshGeometry->IndexCpu->GetBufferPointer(), indices.data(), indexBufferSize);

			meshGeometry->IndexGpu = Utils::CreateDefaultResource(mDevice.Get(), mGraphicsCommandList.Get(),
				indices.data(), indexBufferSize, meshGeometry->IndexGpuUploader);

			meshGeometry->VertexBufferByteSize = vertexBufferSize;
			meshGeometry->VertexByteStride = sizeof(Shader::TextVertex);

			meshGeometry->IndexFormat = DXGI_FORMAT_R32_UINT;
			meshGeometry->IndexBufferByteSize = indexBufferSize;

			Shader::SubmeshGeometry subMeshGeo = {};
			subMeshGeo.IndexCount = (UINT)indices.size();
			subMeshGeo.BaseVertexLocation = 0;
			subMeshGeo.StartIndexLocation = 0;

			meshGeometry->DrawArgs["text"] = subMeshGeo;

			mDrawArgs[meshGeometry->name] = std::move(meshGeometry);
		}

		void BuildRenderItems()
		{
			auto quadRitem1 = std::make_unique<Shader::RenderItem>();
			quadRitem1->ObjIndex = 0;
			DirectX::XMStoreFloat4x4(&quadRitem1->World, DirectX::XMMatrixScaling(mHalfSize.x * 2.0f, mHalfSize.y * 2.0f, mHalfSize.z * 2.0f));
			quadRitem1->TexTransform = Utils::MatrixIdentity();
			quadRitem1->Mat = mMaterials["wood"].get();
			quadRitem1->meshGeo = mDrawArgs["shapeGeo"].get();
			quadRitem1->IndexCount = quadRitem1->meshGeo->DrawArgs["box"].IndexCount;
			quadRitem1->BaseVertexLocation = quadRitem1->meshGeo->DrawArgs["box"].BaseVertexLocation;
			quadRitem1->StartIndexLocation = quadRitem1->meshGeo->DrawArgs["box"].StartIndexLocation;
			quadRitem1->PrimitiveTopology = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

			mBox1VB = quadRitem1.get();
			mRenderLayer[(size_t)Shader::RenderLayer::Opaque].push_back(quadRitem1.get());
			mAllRenderItems.push_back(std::move(quadRitem1));

			auto quadRitem2 = std::make_unique<Shader::RenderItem>();
			quadRitem2->ObjIndex = 1;
			DirectX::XMStoreFloat4x4(&quadRitem2->World, DirectX::XMMatrixScaling(mHalfSize.x * 2.0f, mHalfSize.y * 2.0f, mHalfSize.z * 2.0f));
			quadRitem2->TexTransform = Utils::MatrixIdentity();
			quadRitem2->Mat = mMaterials["wood"].get();
			quadRitem2->meshGeo = mDrawArgs["shapeGeo"].get();
			quadRitem2->IndexCount = quadRitem2->meshGeo->DrawArgs["box"].IndexCount;
			quadRitem2->BaseVertexLocation = quadRitem2->meshGeo->DrawArgs["box"].BaseVertexLocation;
			quadRitem2->StartIndexLocation = quadRitem2->meshGeo->DrawArgs["box"].StartIndexLocation;
			quadRitem2->PrimitiveTopology = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

			mBox2VB = quadRitem2.get();
			mRenderLayer[(size_t)Shader::RenderLayer::Opaque].push_back(quadRitem2.get());
			mAllRenderItems.push_back(std::move(quadRitem2));


			auto quadRitem3 = std::make_unique<Shader::RenderItem>();
			quadRitem3->ObjIndex = 2;
			DirectX::XMStoreFloat4x4(&quadRitem3->World, DirectX::XMMatrixScaling(mHalfSize.x * 2.0f, mHalfSize.y * 2.0f, mHalfSize.z * 2.0f));
			quadRitem3->TexTransform = Utils::MatrixIdentity();
			quadRitem3->Mat = mMaterials["wood"].get();
			quadRitem3->meshGeo = mDrawArgs["shapeGeo"].get();
			quadRitem3->IndexCount = quadRitem3->meshGeo->DrawArgs["box"].IndexCount;
			quadRitem3->BaseVertexLocation = quadRitem3->meshGeo->DrawArgs["box"].BaseVertexLocation;
			quadRitem3->StartIndexLocation = quadRitem3->meshGeo->DrawArgs["box"].StartIndexLocation;
			quadRitem3->PrimitiveTopology = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

			mBox3VB = quadRitem3.get();
			mRenderLayer[(size_t)Shader::RenderLayer::Opaque].push_back(quadRitem3.get());
			mAllRenderItems.push_back(std::move(quadRitem3));


			auto textRitem = std::make_unique<Shader::RenderItem>();
			textRitem->ObjIndex = 3;
			textRitem->TexTransform = Utils::MatrixIdentity();
			textRitem->Mat = mMaterials["text"].get();
			textRitem->meshGeo = mDrawArgs["textGeo"].get();
			textRitem->IndexCount = textRitem->meshGeo->DrawArgs["text"].IndexCount;
			textRitem->BaseVertexLocation = textRitem->meshGeo->DrawArgs["text"].BaseVertexLocation;
			textRitem->StartIndexLocation = textRitem->meshGeo->DrawArgs["text"].StartIndexLocation;
			textRitem->PrimitiveTopology = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

			mTextVB = textRitem.get();
			mRenderLayer[(size_t)Shader::RenderLayer::Text].push_back(textRitem.get());
			mAllRenderItems.push_back(std::move(textRitem));
		}

		void BuildRigidBodies()
		{
			mCData.contactArray = mContact;

			mRigidBody1.setCanSleep(false);
			mRigidBody1.setAwake(true);
			mRigidBody1.setMass(8.0f);
			mRigidBody1.setVelocity(Vector3(0.0f, 0.0f, 0.0f));
			mRigidBody1.setAcceleration(Vector3(0.0f, -10.0f, 0.0f));
			mRigidBody1.setPosition(Vector3(1.2f, 13.0f, 10.0f));
			mRigidBody1.setDamping(0.95f, 0.8f);
			mRigidBody1.setRotation(Vector3(0.0f, 0.0f, 0.0f));
			mRigidBody1.setOrientation(Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
			Matrix3 inertiaTensor;
			inertiaTensor.setBlockInertiaTensor(mHalfSize, mHalfSize.x * mHalfSize.y * mHalfSize.z * 8.0f);
			mRigidBody1.setInertiaTensor(inertiaTensor);

			mRigidBody2.setCanSleep(false);
			mRigidBody2.setAwake(true);
			mRigidBody2.setMass(8.0f);
			mRigidBody2.setVelocity(Vector3(0.0f, 0.0f, 0.0f));
			mRigidBody2.setAcceleration(Vector3(0.0f, -10.0f, 0.0f));
			mRigidBody2.setPosition(Vector3(0.0f, 0.0f, 10.0f));
			mRigidBody2.setDamping(0.95f, 0.8f);
			mRigidBody2.setRotation(Vector3(0.0f, 0.0f, 0.0f));
			mRigidBody2.setOrientation(Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
			mRigidBody2.setInertiaTensor(inertiaTensor);

			mRigidBody3.setCanSleep(false);
			mRigidBody3.setAwake(true);
			mRigidBody3.setMass(8.0f);
			mRigidBody3.setVelocity(Vector3(0.0f, 40.0f, 0.0f));
			mRigidBody3.setAcceleration(Vector3(0.0f, -10.0f, 0.0f));
			mRigidBody3.setPosition(Vector3(0.5f, -40.0f, 10.0f));
			mRigidBody3.setDamping(0.95f, 0.8f);
			mRigidBody3.setRotation(Vector3(0.0f, 0.0f, 0.0f));
			mRigidBody3.setOrientation(Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
			mRigidBody3.setInertiaTensor(inertiaTensor);


			mContactResolver = std::make_unique<ContactResolver>(256 * 8);

			UpdateRigidBodies(0.00001f);
			isSimulating = false;
		}

		void BuildFrameResources()
		{
			for (int i = 0; i < Shader::gNumFrameResources; ++i)
			{
				mFrameResources.push_back(std::make_unique<FrameResource>(mDevice.Get(),
					1, (UINT)mAllRenderItems.size(), 1, gMaxNumTextCharacters * 4));
			}
		}

		void BuildPSOs()
		{
			D3D12_GRAPHICS_PIPELINE_STATE_DESC graphicsPSODescriptor = {};
			graphicsPSODescriptor.NodeMask = 0;
			graphicsPSODescriptor.InputLayout = { mInputLayouts["layout1"].data(), (UINT)mInputLayouts["layout1"].size() };
			graphicsPSODescriptor.pRootSignature = mRootSignatrue.Get();
			graphicsPSODescriptor.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
			graphicsPSODescriptor.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
			graphicsPSODescriptor.DSVFormat = gDepthStencilFormat;
			graphicsPSODescriptor.NumRenderTargets = 1;
			graphicsPSODescriptor.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
			graphicsPSODescriptor.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
			graphicsPSODescriptor.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
			graphicsPSODescriptor.RTVFormats[0] = gBackBufferFormat;
			graphicsPSODescriptor.SampleMask = UINT_MAX;
			graphicsPSODescriptor.SampleDesc = { 1, 0 };
			graphicsPSODescriptor.VS = {
				reinterpret_cast<BYTE*>(mShaders["vs"]->GetBufferPointer()),
				mShaders["vs"]->GetBufferSize()
			};
			graphicsPSODescriptor.PS = {
				reinterpret_cast<BYTE*>(mShaders["ps"]->GetBufferPointer()),
				mShaders["ps"]->GetBufferSize()
			};

			HR(mDevice->CreateGraphicsPipelineState(&graphicsPSODescriptor, IID_PPV_ARGS(mGraphicsPSO["opaque"].GetAddressOf())));

			D3D12_GRAPHICS_PIPELINE_STATE_DESC textPSODescriptor = graphicsPSODescriptor;
			textPSODescriptor.DepthStencilState.DepthEnable = TRUE;
			textPSODescriptor.BlendState.AlphaToCoverageEnable = FALSE;
			textPSODescriptor.BlendState.IndependentBlendEnable = FALSE;
			textPSODescriptor.BlendState.RenderTarget[0].BlendEnable = TRUE;

			textPSODescriptor.BlendState.RenderTarget[0].SrcBlend = D3D12_BLEND_ONE;
			textPSODescriptor.BlendState.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
			textPSODescriptor.BlendState.RenderTarget[0].DestBlend = D3D12_BLEND_SRC_ALPHA;
			textPSODescriptor.BlendState.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ZERO;
			textPSODescriptor.BlendState.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
			textPSODescriptor.BlendState.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
			textPSODescriptor.BlendState.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

			textPSODescriptor.DepthStencilState.DepthEnable = false;
			textPSODescriptor.DepthStencilState.DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL;
			textPSODescriptor.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS;
			textPSODescriptor.DepthStencilState.StencilEnable = true;
			textPSODescriptor.DepthStencilState.StencilReadMask = 0xFF;
			textPSODescriptor.DepthStencilState.StencilWriteMask = 0xFF;
			textPSODescriptor.DepthStencilState.FrontFace.StencilFailOp = D3D12_STENCIL_OP_KEEP;
			textPSODescriptor.DepthStencilState.FrontFace.StencilDepthFailOp = D3D12_STENCIL_OP_INCR;
			textPSODescriptor.DepthStencilState.FrontFace.StencilPassOp = D3D12_STENCIL_OP_KEEP;
			textPSODescriptor.DepthStencilState.FrontFace.StencilFunc = D3D12_COMPARISON_FUNC_ALWAYS;
			textPSODescriptor.DepthStencilState.BackFace.StencilFailOp = D3D12_STENCIL_OP_KEEP;
			textPSODescriptor.DepthStencilState.BackFace.StencilDepthFailOp = D3D12_STENCIL_OP_DECR;
			textPSODescriptor.DepthStencilState.BackFace.StencilPassOp = D3D12_STENCIL_OP_KEEP;
			textPSODescriptor.DepthStencilState.BackFace.StencilFunc = D3D12_COMPARISON_FUNC_ALWAYS;

			textPSODescriptor.InputLayout = { mInputLayouts["layout2"].data(), (UINT)mInputLayouts["layout2"].size() };
			textPSODescriptor.VS = {
				reinterpret_cast<BYTE*>(mShaders["text_vs"]->GetBufferPointer()),
				mShaders["text_vs"]->GetBufferSize()
			};
			textPSODescriptor.PS = {
				reinterpret_cast<BYTE*>(mShaders["text_ps"]->GetBufferPointer()),
				mShaders["text_ps"]->GetBufferSize()
			};

			HR(mDevice->CreateGraphicsPipelineState(&textPSODescriptor, IID_PPV_ARGS(mGraphicsPSO["text"].GetAddressOf())));
		}

		void CollisionDetection()
		{
			if (!mCData.hasMoreContacts())
				return;

			CollisionDetector::boxAndBox(mCBox1, mCBox2, &mCData);
			CollisionDetector::boxAndBox(mCBox2, mCBox3, &mCData);
			CollisionDetector::boxAndBox(mCBox3, mCBox1, &mCData);
		}

		inline DirectX::XMFLOAT4X4 real16ToFloat4x4(const float* m)
		{
			DirectX::XMFLOAT4X4 result;

			result.m[0][0] = m[0];	result.m[0][1] = m[1];	result.m[0][2] = m[2];	result.m[0][3] = m[3];
			result.m[1][0] = m[4];	result.m[1][1] = m[5];	result.m[1][2] = m[6];	result.m[1][3] = m[7];
			result.m[2][0] = m[8];	result.m[2][1] = m[9];	result.m[2][2] = m[10]; result.m[2][3] = m[11];
			result.m[3][0] = m[12]; result.m[3][1] = m[13]; result.m[3][2] = m[14]; result.m[3][3] = m[15];

			return result;
		}

		void UpdateRigidBodies(float dt)
		{
			if ((GetAsyncKeyState('Z') & 0x8000) != 0)
				isSimulating = false;
			if ((GetAsyncKeyState('X') & 0x8000) != 0)
				isSimulating = true;

			if (isSimulating)
			{
				// duration
				float duration = dt;

				// integrate
				mRigidBody1.integrate(duration);

				float tempMatrix[16] = {};
				mRigidBody1.getOTransform(tempMatrix);
				mBox1VB->World = real16ToFloat4x4(tempMatrix);
				mBox1VB->NumframeDirty = Shader::gNumFrameResources;

				mRigidBody2.integrate(duration);

				mRigidBody2.getOTransform(tempMatrix);
				mBox2VB->World = real16ToFloat4x4(tempMatrix);
				mBox2VB->NumframeDirty = Shader::gNumFrameResources;

				mRigidBody3.integrate(duration);

				mRigidBody3.getOTransform(tempMatrix);
				mBox3VB->World = real16ToFloat4x4(tempMatrix);
				mBox3VB->NumframeDirty = Shader::gNumFrameResources;

				// generate contacts
				mCData.reset(256);
				mCData.friction = 0.9f;
				mCData.restitution = 0.1f;
				mCData.tolerance = 0.1f;

				// generate collision boxes
				mCBox1.halfSize = mHalfSize;
				mCBox1.body = &mRigidBody1;
				mCBox1.calculateInternals();

				mCBox2.halfSize = mHalfSize;
				mCBox2.body = &mRigidBody2;
				mCBox2.calculateInternals();

				mCBox3.halfSize = mHalfSize;
				mCBox3.body = &mRigidBody3;
				mCBox3.calculateInternals();

				// collision detection
				CollisionDetection();

				// contact resolve
				mContactResolver->resolveContact(mCData.contactArray, mCData.contactCount, duration);
			}
		}

		void UpdateObjectCB()
		{
			for (auto& e : mAllRenderItems)
			{
				if (e->NumframeDirty > 0)
				{
					ObjectConstants objConstants = {};
					DirectX::XMStoreFloat4x4(&objConstants.World, DirectX::XMMatrixTranspose(DirectX::XMLoadFloat4x4(&e->World)));
					DirectX::XMStoreFloat4x4(&objConstants.TexTransform, DirectX::XMMatrixTranspose(DirectX::XMLoadFloat4x4(&e->TexTransform)));
					objConstants.MatIndex = e->Mat->SrvTextureIndex;

					mCurrFrameResource->mObjCB->CopyData(e->ObjIndex, objConstants);

					--(e->NumframeDirty);
				}
			}
		}

		void UpdatePassCB()
		{
			DirectX::XMMATRIX View = DirectX::XMLoadFloat4x4(&mCamera.View());
			DirectX::XMMATRIX Proj = DirectX::XMLoadFloat4x4(&mCamera.Projection());
			DirectX::XMMATRIX ViewProj = DirectX::XMMatrixMultiply(View, Proj);

			DirectX::XMMATRIX ViewOrtho = DirectX::XMMatrixMultiply(
				DirectX::XMMatrixLookAtLH(DirectX::XMVectorSet(0.0f, 0.0f, -10.0f, 1.0f),
					DirectX::XMVectorSet(0.0f, 0.0f, 1.0f, 1.0f)
				, DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 1.0f)),
				DirectX::XMMatrixOrthographicLH((float)mClientWidth, (float)mClientHeight, 1.0f, 1000.0f));

			PassConstants passConstants = {};
			DirectX::XMStoreFloat4x4(&passConstants.View, DirectX::XMMatrixTranspose(View));
			DirectX::XMStoreFloat4x4(&passConstants.InvView, DirectX::XMMatrixTranspose(DirectX::XMMatrixInverse(&DirectX::XMMatrixDeterminant(View), View)));
			DirectX::XMStoreFloat4x4(&passConstants.Proj, DirectX::XMMatrixTranspose(Proj));
			DirectX::XMStoreFloat4x4(&passConstants.InvProj, DirectX::XMMatrixTranspose(DirectX::XMMatrixInverse(&DirectX::XMMatrixDeterminant(Proj), Proj)));
			DirectX::XMStoreFloat4x4(&passConstants.ViewProj, DirectX::XMMatrixTranspose(ViewProj));
			DirectX::XMStoreFloat4x4(&passConstants.InvViewProj, DirectX::XMMatrixTranspose(DirectX::XMMatrixInverse(&DirectX::XMMatrixDeterminant(ViewProj), ViewProj)));
			DirectX::XMStoreFloat4x4(&passConstants.ViewOrtho, DirectX::XMMatrixTranspose(ViewOrtho));
			passConstants.EyePos = { 0.0f, 0.0f, 0.0f };
			passConstants.DeltaTime = mGameTimer.DeltaTime();
			passConstants.TotalTime = mGameTimer.TotalTime();

			passConstants.AmbientLight = { 0.25f, 0.25f, 0.35f, 1.0f };
			passConstants.Lights[0].Direction = { 0.57735f, -0.57735f, 0.57735f };
			passConstants.Lights[0].Strength = { 0.6f, 0.6f, 0.6f };
			passConstants.Lights[1].Direction = { -0.57735f, -0.57735f, 0.57735f };
			passConstants.Lights[1].Strength = { 0.3f, 0.3f, 0.3f };
			passConstants.Lights[2].Direction = { 0.0f, -0.707f, -0.707f };
			passConstants.Lights[2].Strength = { 0.15f, 0.15f, 0.15f };

			passConstants.Color = { 0.0f, 1.0f, 1.0f, 1.0f };

			mCurrFrameResource->mPassCB->CopyData(0, passConstants);
		}

		void UpdateTextVB()
		{
			using Shader::TextVertex;
			
			DirectX::XMFLOAT4 Acc = {};
			 std::string sentence = "GameTime: " + std::to_string(mGameTimer.TotalTime()) + " seconds";
			int numLetters = (int)sentence.size();

			if (numLetters >= gMaxNumTextCharacters)
			{
				throw std::runtime_error("sentence >= gMaxNumTextCharacters");
			}

			float positionX = 10.0f, positionY = 10.0f;
			std::vector<TextVertex> vertices(numLetters * 4);
			float drawX = (float)(((float)mClientWidth / 2.0f) * -1.0f) + positionX;
			float drawY = (float)((float)mClientHeight / 2.0f) - positionY;
			BuildVertexArray(mFontData["original"], vertices.data(), sentence.c_str(), drawX, drawY, 3.0f, 32.0f);

			for (size_t i = 0; i < vertices.size(); ++i)
			{
				mCurrFrameResource->mTextVB->CopyData((UINT)i, vertices[i]);
			}


			mTextVB->meshGeo->VertexGpu = mCurrFrameResource->mTextVB->Resource();
			mTextVB->NumframeDirty = Shader::gNumFrameResources;
		}

		void DrawRitems(const std::vector<Shader::RenderItem*>& rItems)
		{
			UINT cbPerObjectSize = Utils::ConstantBufferSize(sizeof(ObjectConstants));

			for (auto& r : rItems)
			{
				mGraphicsCommandList->IASetVertexBuffers(0, 1, &r->meshGeo->VertexBufferView());
				mGraphicsCommandList->IASetIndexBuffer(&r->meshGeo->IndexBufferView());
				mGraphicsCommandList->IASetPrimitiveTopology(r->PrimitiveTopology);

				mGraphicsCommandList->SetGraphicsRootConstantBufferView(0, r->ObjIndex * cbPerObjectSize + mCurrFrameResource->mObjCB->Resource()->GetGPUVirtualAddress());
				mGraphicsCommandList->DrawIndexedInstanced(r->IndexCount, 1, r->StartIndexLocation, r->BaseVertexLocation, 0);
			}
		}

		void UpdateInput(float dt)
		{
			const float speed = 15.0f;

			if ((GetAsyncKeyState('W') & 0x8000) != 0)
				mCamera.Walk(speed * dt);
			if ((GetAsyncKeyState('S') & 0x8000) != 0)
				mCamera.Walk(-speed * dt);
			if ((GetAsyncKeyState('D') & 0x8000) != 0)
				mCamera.Strafe(speed * dt);
			if ((GetAsyncKeyState('A') & 0x8000) != 0)
				mCamera.Strafe(-speed * dt);

			mCamera.UpdateViewMatrix();
		}

	protected:
		virtual void init() override
		{
			mCamera.SetPosition(0.0f, 0.0f, -20.0f);

			HR(mGraphicsCommandList->Reset(mCommandAllocator.Get(), nullptr));

			BuildTexture();
			BuildRootSignature();
			BuildDescriptor();
			BuildShadersAndInputLayout();
			BuildMaterial();
			BuildShapeMesh();
			BuildFont();
			BuildRenderItems();
			BuildRigidBodies();
			BuildFrameResources();
			BuildPSOs();

			HR(mGraphicsCommandList->Close());
			ID3D12CommandList* cmdLists[] = { mGraphicsCommandList.Get() };
			mCommandQueue->ExecuteCommandLists(_countof(cmdLists), cmdLists);

			FlushCommandList();
		}

		virtual void update(float dt) override
		{
			UpdateInput(dt);

			mCurrFrameResourceIndex = (mCurrFrameResourceIndex + 1) % Shader::gNumFrameResources;
			mCurrFrameResource = mFrameResources[mCurrFrameResourceIndex].get();

			if (mFence->GetCompletedValue() < mCurrFrameResource->mFenceCount && mCurrFrameResource->mFenceCount != 0)
			{
				HANDLE hEvent = CreateEventEx(nullptr, nullptr, 0, EVENT_ALL_ACCESS);
				mFence->SetEventOnCompletion(mCurrFrameResource->mFenceCount, hEvent);
				WaitForSingleObject(hEvent, 0xffffffff);
				CloseHandle(hEvent);
			}

			UpdateRigidBodies(dt);
			UpdateObjectCB();
			UpdatePassCB();
			UpdateTextVB();
		}

		virtual void draw(float dt) override
		{
			HR(mCommandAllocator->Reset());
			HR(mGraphicsCommandList->Reset(mCommandAllocator.Get(), mGraphicsPSO["opaque"].Get()));

			mGraphicsCommandList->ResourceBarrier(1,
				&unmove(CD3DX12_RESOURCE_BARRIER::Transition(mBackBuffer[mCurrBackBufferIndex].Get(),
					D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET)));

			mGraphicsCommandList->SetGraphicsRootSignature(mRootSignatrue.Get());
			ID3D12DescriptorHeap* descriptorHeaps[] = { mSrvHeap.Get() };
			mGraphicsCommandList->SetDescriptorHeaps(_countof(descriptorHeaps), descriptorHeaps);
			mGraphicsCommandList->SetGraphicsRootConstantBufferView(1, mCurrFrameResource->mPassCB->Resource()->GetGPUVirtualAddress());
			CD3DX12_GPU_DESCRIPTOR_HANDLE srvHandle(mSrvHeap->GetGPUDescriptorHandleForHeapStart());
			mGraphicsCommandList->SetGraphicsRootDescriptorTable(2, srvHandle);


			auto rtv = rtvHandle();
			auto dsv = dsvHandle();

			FLOAT colors[4] = { 0.2f, 0.2f, 0.4f, 1.0f };
			mGraphicsCommandList->ClearRenderTargetView(rtv, colors, 0, nullptr);
			mGraphicsCommandList->ClearDepthStencilView(dsv, D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);
			mGraphicsCommandList->OMSetRenderTargets(1, &rtv, true, &dsv);

			mGraphicsCommandList->RSSetScissorRects(1, &mScissorRect);
			mGraphicsCommandList->RSSetViewports(1, &mViewPort);

			DrawRitems(mRenderLayer[(int)Shader::RenderLayer::Opaque]);
			mGraphicsCommandList->SetPipelineState(mGraphicsPSO["text"].Get());
			DrawRitems(mRenderLayer[(int)Shader::RenderLayer::Text]);

			mGraphicsCommandList->ResourceBarrier(1,
				&unmove(CD3DX12_RESOURCE_BARRIER::Transition(mBackBuffer[mCurrBackBufferIndex].Get(),
					D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT)));

			mGraphicsCommandList->Close();
			ID3D12CommandList* cmdLists[] = { mGraphicsCommandList.Get() };
			mCommandQueue->ExecuteCommandLists(_countof(cmdLists), cmdLists);

			mSwapChain->Present(0, 0);
			mCurrBackBufferIndex = (mCurrBackBufferIndex + 1) % gBackBufferCount;

			// FlushCommandList();

			mCurrFrameResource->mFenceCount = ++mCurrFenceCount;
			mCommandQueue->Signal(mFence.Get(), mCurrFenceCount);
		}

		virtual void OnResize(UINT screenWidth, UINT screenHeight) override
		{
			Renderer::OnResize(screenWidth, screenHeight);

			mCamera.SetLens(0.25f * DirectX::XM_PI, (float)mClientWidth / (float)mClientHeight, 1.0f, 1000.0f);
		}

		virtual void RButtonDown(WPARAM btnState, int x, int y) override
		{
			mLastMousePosX = x;
			mLastMousePosY = y;

			SetCapture(mHwnd);
		}

		virtual void RButtonUp(WPARAM btnState, int x, int y) override
		{
			ReleaseCapture();
		}

		virtual void MouseMove(WPARAM btnState, int x, int y) override
		{
			if ((btnState & MK_LBUTTON) != 0)
			{
				float dx = DirectX::XMConvertToRadians(0.25f * static_cast<float>(x - mLastMousePosX));
				float dy = DirectX::XMConvertToRadians(0.25f * static_cast<float>(y - mLastMousePosY));

				mCamera.Pitch(dy);
				mCamera.RotationY(dx);
			}

			mLastMousePosX = x;
			mLastMousePosY = y;
		}
	};
}