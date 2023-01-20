#include <cstdint>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <Misc/SizedTypes.h>
#include <Misc/SelfDestructPointer.h>
#include <Misc/FixedArray.h>
#include <Misc/FunctionCalls.h>
#include <Misc/MessageLogger.h>
#include <Misc/FileNameExtensions.h>
#include <Misc/StandardValueCoders.h>
#include <Misc/ArrayValueCoders.h>
#include <Misc/ConfigurationFile.h>
#include <IO/File.h>
#include <IO/ValueSource.h>
#include <IO/OpenFile.h>
#include <Comm/OpenPipe.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Math/Interval.h>
#include <Math/MathValueCoders.h>
#include <Geometry/Point.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/HVector.h>
#include <Geometry/Plane.h>
#include <Geometry/LinearUnit.h>
#include <Geometry/GeometryValueCoders.h>
#include <Geometry/OutputOperators.h>
#include <GL/gl.h>
#include <GL/GLMaterialTemplates.h>
#include <GL/GLColorMap.h>
#include <GL/GLLightTracker.h>
#include <GL/Extensions/GLEXTFramebufferObject.h>
#include <GL/Extensions/GLARBTextureRectangle.h>
#include <GL/Extensions/GLARBTextureFloat.h>
#include <GL/Extensions/GLARBTextureRg.h>
#include <GL/Extensions/GLARBDepthTexture.h>
#include <GL/Extensions/GLARBShaderObjects.h>
#include <GL/Extensions/GLARBVertexShader.h>
#include <GL/Extensions/GLARBFragmentShader.h>
#include <GL/Extensions/GLARBMultitexture.h>
#include <GL/GLContextData.h>
#include <GL/GLGeometryWrappers.h>
#include <GL/GLTransformationWrappers.h>
#include <GLMotif/StyleSheet.h>
#include <GLMotif/WidgetManager.h>
#include <GLMotif/PopupMenu.h>
#include <GLMotif/Menu.h>
#include <GLMotif/PopupWindow.h>
#include <GLMotif/Margin.h>
#include <GLMotif/Label.h>
#include <GLMotif/TextField.h>
#include <Vrui/Vrui.h>
#include <Vrui/CoordinateManager.h>
#include <Vrui/Lightsource.h>
#include <Vrui/LightsourceManager.h>
#include <Vrui/Viewer.h>
#include <Vrui/ToolManager.h>
#include <Vrui/DisplayState.h>
#include <Kinect/FileFrameSource.h>
#include <Kinect/MultiplexedFrameSource.h>
#include <Kinect/DirectFrameSource.h>
#include <Kinect/OpenDirectFrameSource.h>

#include "Types.h"
#include "FrameFilter.h"
#include "DepthImageRenderer.h"
#include "ElevationColorMap.h"
#include "DEM.h"
#include "SurfaceRenderer.h"
#include "WaterTable2.h"
#include "HandExtractor.h"
#include "RemoteServer.h"
#include "WaterRenderer.h"
//#include "GlobalWaterTool.h"
//#include "LocalWaterTool.h"
//#include "DEMTool.h"
//#include "BathymetrySaverTool.h"


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Skybox.h>
#include <Urho3D/Graphics/Terrain.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/GraphicsAPI/RenderSurface.h>
#include <Urho3D/GraphicsAPI/Texture2D.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/Resource/Image.h>
#include <Urho3D/UI/UI.h>

#include "TerrainSandbox.h"

#include <Urho3D/DebugNew.h>
#include "FpsCounter.h"

typedef Kinect::FrameSource::DepthCorrection::PixelCorrection PixelDepthCorrection; // Type for per-pixel depth correction factors
typedef uint16_t DepthPixel; // Type for per-pixel depth correction factors

struct Config {
    std::string videoSource = "";
    std::string kinectServer = ""; //"0.0.0.0";
    int cameraIndex = 0;
    int numAveragingSlots = 30;
};


Kinect::FrameSource* camera;                                // The Kinect camera device
unsigned int frameSize[2];                                  // Width and height of the camera's depth frames
PixelDepthCorrection* pixelDepthCorrection;                 // Buffer of per-pixel depth correction coefficients
Kinect::FrameSource::IntrinsicParameters cameraIps;         // Intrinsic parameters of the Kinect camera
FrameFilter* frameFilter;                                   // Processing object to filter raw depth frames from the Kinect camera
Threads::TripleBuffer<Kinect::FrameBuffer> filteredFrames;  // Triple buffer for incoming filtered depth frames
DepthImageRenderer* depthImageRenderer;                     // Object managing the current filtered depth image
HandExtractor* handExtractor;                               // Object to detect splayed hands above the sand surface to make rain

Terrain *terrain;
bool setEvent = false;
Image *terrainImage = nullptr;
Texture2D* logoTexture = nullptr;
FpsCounter fpsCounter;
Text *fpsElement;

const String CURRENT_FPS_STR = "Current FPS";

void rawDepthFrameDispatcher(const Kinect::FrameBuffer &frameBuffer) {
    /* Pass the received frame to the frame filter and the hand extractor: */
//    if (frameFilter != 0 && !pauseUpdates)
//        frameFilter->receiveRawFrame(frameBuffer);
//    if (handExtractor != 0)
//        handExtractor->receiveRawFrame(frameBuffer);

    const auto *data = frameBuffer.getData<DepthPixel>();
    unsigned height = frameBuffer.getSize(0);
    unsigned width = frameBuffer.getSize(1);
//
//    if (setEvent) {
//        setEvent = false;
//        std::cout << "received frame." << std::endl;
//    cv::Mat image16 = cv::Mat(width, height, CV_16UC1, (void*)data);
        unsigned char data2[width * height];
        for (int i = 0; i < width * height; i++) {
            int d = (data[i] / 256) * 20;
            if (d > 255)
                d = 255;
            data2[i] = d;
        }

        terrainImage->SetSize(width, height, 1);
        terrainImage->SetData(data2);

        //terrain->SetHeightMap(image);
//    }
//    cv::Mat image = cv::Mat(width, height, CV_16UC1, (void*)data);
//    for (int i = 0; i < width; i++) {
//        for (int j = 0; j < height; j++) {
//            image.at<DepthPixel>(i, j) = image.at<DepthPixel>(i, j) * 10 + 20;
//        }
//    }
//
//    cv::imshow("data", image);
//    cv::waitKey();
}




URHO3D_DEFINE_APPLICATION_MAIN(Water)

Water::Water(Context* context) :
        Sample(context)
{
}

void Water::CreateCurrentFpsUiElement()
{
    UIElement* root = GetSubsystem<UI>()->GetRoot();
    root->SetDefaultStyle(GetSubsystem<ResourceCache>()->GetResource<XMLFile>("UI/DefaultStyle.xml"));

    fpsElement = root->CreateChild<Text>(CURRENT_FPS_STR);
    fpsElement->SetStyleAuto();
    fpsElement->SetTextEffect(TE_SHADOW);
    fpsElement->SetPosition(10, 10);
    fpsElement->SetText("FPS");
}

void Water::UpdateCurrentFpsElement()
{
    String fpsStr = fpsCounter.GetCurrentFps() == -1 ? "?" : String(fpsCounter.GetCurrentFps());
    fpsElement->SetText("FPS: " + fpsStr);
}

void Water::Start()
{
    // Execute base class startup
    Sample::Start();

    // Create the scene content
    CreateScene();

    // Create the UI content
    CreateInstructions();

    CreateCurrentFpsUiElement();

    // Setup the viewport for displaying the scene
    SetupViewport();

    // Hook up to the frame update event
    SubscribeToEvents();

    // Set the mouse mode to use in the sample
    Sample::InitMouseMode(MM_RELATIVE);

    terrainImage = new Image(context_);

    Config config;

    /* Create Camera object from Kinect source or prerecorded video. */
    if (!config.videoSource.empty()) {
        std::string colorFileName = config.videoSource + ".color";
        std::string depthFileName = config.videoSource + ".depth";
        camera = new Kinect::FileFrameSource(
                IO::openFile(colorFileName.c_str()),
                IO::openFile(depthFileName.c_str())
        );
    }
    else if (!config.kinectServer.empty()) {
        /* Split the server name into host name and port: */
        const char *colonPos;
        for (colonPos = config.kinectServer.c_str(); *colonPos != '\0'; ++colonPos)
            if (*colonPos == ':')
                break;

        std::string hostName;
        int port;

        if (*colonPos == ':') {
            hostName = std::string(config.kinectServer.c_str(), colonPos);
            port = atoi(colonPos + 1);
        } else {
            /* Use complete host name and default port: */
            hostName = config.kinectServer;
            port = 26000;
        }

        /* Open a multiplexed frame source for the given server host name and port number: */
        Kinect::MultiplexedFrameSource *source = Kinect::MultiplexedFrameSource::create(
                Comm::openTCPPipe(hostName.c_str(), port));

        /* Use the server's first component stream as the camera device: */
        camera = source->getStream(0);
    } else {
        /* Open the 3D camera device of the selected index: */
        Kinect::DirectFrameSource *realCamera = Kinect::openDirectFrameSource(config.cameraIndex, false);
        //Misc::ConfigurationFileSection cameraConfigurationSection = cfg.getSection(cameraConfiguration.c_str());
        //realCamera->configure(cameraConfigurationSection);
        camera = realCamera;
    }

    for (int i = 0; i < 2; ++i)
        frameSize[i] = camera->getActualFrameSize(Kinect::FrameSource::DEPTH)[i];

    /* Get the camera's per-pixel depth correction parameters and evaluate it on the depth frame's pixel grid: */
    Kinect::FrameSource::DepthCorrection *depthCorrection = camera->getDepthCorrectionParameters();
    if (depthCorrection != 0) {
        pixelDepthCorrection = depthCorrection->getPixelCorrection(frameSize);
        delete depthCorrection;
    } else {
        /* Create dummy per-pixel depth correction parameters: */
        pixelDepthCorrection = new PixelDepthCorrection[frameSize[1] * frameSize[0]];
        PixelDepthCorrection *pdcPtr = pixelDepthCorrection;
        for (unsigned int y = 0; y < frameSize[1]; ++y)
            for (unsigned int x = 0; x < frameSize[0]; ++x, ++pdcPtr) {
                pdcPtr->scale = 1.0f;
                pdcPtr->offset = 0.0f;
            }
    }

    /* Get the camera's intrinsic parameters: */
    cameraIps = camera->getIntrinsicParameters();

//    frameFilter = new FrameFilter(frameSize, config.numAveragingSlots, pixelDepthCorrection, cameraIps.depthProjection,
//                                  basePlane);
//    frameFilter->setValidElevationInterval(cameraIps.depthProjection, basePlane, elevationRange.getMin(),
//                                           elevationRange.getMax());
//    frameFilter->setStableParameters(minNumSamples, maxVariance);
//    frameFilter->setHysteresis(hysteresis);
//    frameFilter->setSpatialFilter(true);
//    frameFilter->setOutputFrameFunction(Misc::createFunctionCall(this, &Sandbox::receiveFilteredFrame));


    handExtractor = new HandExtractor(frameSize, pixelDepthCorrection, cameraIps.depthProjection);

    camera->startStreaming(0, Misc::createFunctionCall(rawDepthFrameDispatcher));

    /* Create the depth image renderer: */
    depthImageRenderer = new DepthImageRenderer(frameSize);
    depthImageRenderer->setIntrinsics(cameraIps);
//    depthImageRenderer->setBasePlane(basePlane);


    std::cout << "Initialization done." << std::endl;
    fpsCounter.Clear();
}

void Water::CreateScene()
{
    auto* cache = GetSubsystem<ResourceCache>();

    scene_ = new Scene(context_);

    // Create octree, use default volume (-1000, -1000, -1000) to (1000, 1000, 1000)
    scene_->CreateComponent<Octree>();

    // Create a Zone component for ambient lighting & fog control
    Node* zoneNode = scene_->CreateChild("Zone");
    auto* zone = zoneNode->CreateComponent<Zone>();
    zone->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
    zone->SetAmbientColor(Color(0.15f, 0.15f, 0.15f));
    zone->SetFogColor(Color(1.0f, 1.0f, 1.0f));
    zone->SetFogStart(500.0f);
    zone->SetFogEnd(750.0f);

    // Create a directional light to the world. Enable cascaded shadows on it
    Node* lightNode = scene_->CreateChild("DirectionalLight");
    lightNode->SetDirection(Vector3(0.6f, -1.0f, 0.8f));
    auto* light = lightNode->CreateComponent<Light>();
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetCastShadows(true);
    light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
    light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
    light->SetSpecularIntensity(0.5f);
    // Apply slightly overbright lighting to match the skybox
    light->SetColor(Color(1.2f, 1.2f, 1.2f));

    // Create skybox. The Skybox component is used like StaticModel, but it will be always located at the camera, giving the
    // illusion of the box planes being far away. Use just the ordinary Box model and a suitable material, whose shader will
    // generate the necessary 3D texture coordinates for cube mapping
    Node* skyNode = scene_->CreateChild("Sky");
    skyNode->SetScale(500.0f); // The scale actually does not matter
    auto* skybox = skyNode->CreateComponent<Skybox>();
    skybox->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    skybox->SetMaterial(cache->GetResource<Material>("Materials/Skybox.xml"));

    // Create heightmap terrain
    Node* terrainNode = scene_->CreateChild("Terrain");
    terrainNode->SetPosition(Vector3(0.0f, 0.0f, 0.0f));
    terrain = terrainNode->CreateComponent<Terrain>();
    terrain->SetPatchSize(64);
    terrain->SetSpacing(Vector3(2.0f, 0.5f, 2.0f)); // Spacing between vertices and vertical resolution of the height map
    terrain->SetSmoothing(true);
    terrain->SetHeightMap(cache->GetResource<Image>("Textures/HeightMap.png"));
    terrain->SetMaterial(cache->GetResource<Material>("Materials/Terrain.xml"));
    // The terrain consists of large triangles, which fits well for occlusion rendering, as a hill can occlude all
    // terrain patches and other objects behind it
    terrain->SetOccluder(true);

    // Create the camera. Set far clip to match the fog. Note: now we actually create the camera node outside
    // the scene, because we want it to be unaffected by scene load / save
    cameraNode_ = new Node(context_);
    auto* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(750.0f);

    // Set an initial position for the camera scene node above the ground
    cameraNode_->SetPosition(Vector3(0.0f, 7.0f, -20.0f));
}

void Water::CreateInstructions()
{
//    auto* cache = GetSubsystem<ResourceCache>();
//    auto* ui = GetSubsystem<UI>();
//
//    // Construct new Text object, set string to display and font to use
//    auto* instructionText = ui->GetRoot()->CreateChild<Text>();
//    instructionText->SetText("Use WASD keys and mouse/touch to move");
//    instructionText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
//    instructionText->SetTextAlignment(HA_CENTER);
//
//    // Position the text relative to the screen center
//    instructionText->SetHorizontalAlignment(HA_CENTER);
//    instructionText->SetVerticalAlignment(VA_CENTER);
//    instructionText->SetPosition(0, ui->GetRoot()->GetHeight() / 4);

    // Get logo texture
//    ResourceCache* cache = GetSubsystem<ResourceCache>();
//    logoTexture = cache->GetResource<Texture2D>("Textures/HeightMap.png");
//    if (!logoTexture)
//        return;



}

void Water::SetupViewport()
{
    auto* graphics = GetSubsystem<Graphics>();
    auto* renderer = GetSubsystem<Renderer>();
    auto* cache = GetSubsystem<ResourceCache>();

    // Set up a viewport to the Renderer subsystem so that the 3D scene can be seen
    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    renderer->SetViewport(0, viewport);
}

void Water::SubscribeToEvents()
{
    // Subscribe HandleUpdate() function for processing update events
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(Water, HandleUpdate));
}

void Water::MoveCamera(float timeStep) {
    // Do not move if the UI has a focused element (the console)
    if (GetSubsystem<UI>()->GetFocusElement())
        return;

    auto *input = GetSubsystem<Input>();

    // Movement speed as world units per second
    const float MOVE_SPEED = 80.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;

    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);

    // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
    cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    if (input->GetKeyDown(KEY_W))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_S))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_A))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_D))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);
//    if (input->GetKeyDown(KEY_E))


    if (terrainImage != nullptr) {
        terrain->SetHeightMap(terrainImage);
        if (logoTexture == nullptr) {
            // Create logo sprite and add to the UI layout
            logoTexture = new Texture2D(context_);
            logoTexture->SetData(terrainImage);

            UI *ui = GetSubsystem<UI>();
            logoSprite_ = ui->GetRoot()->CreateChild<Sprite>();

            // Set logo sprite texture
            logoSprite_->SetTexture(logoTexture);

            int textureWidth = logoTexture->GetWidth();
            int textureHeight = logoTexture->GetHeight();

            std::cout << textureWidth << " " << textureHeight << std::endl;

            // Set logo sprite scale
            logoSprite_->SetScale(256.0f / textureWidth);

            // Set logo sprite size
            logoSprite_->SetSize(textureWidth, textureHeight);

            // Set logo sprite hot spot
            logoSprite_->SetHotSpot(textureWidth, textureHeight);

            // Set logo sprite alignment
            logoSprite_->SetAlignment(HA_RIGHT, VA_BOTTOM);

            // Make logo not fully opaque to show the scene underneath
            logoSprite_->SetOpacity(1.0f);

            // Set a low priority for the logo so that other UI elements can be drawn on top
            logoSprite_->SetPriority(-100);
        } else {
            logoTexture->SetData(terrainImage);
        }
    }
}

void Water::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    // Take the frame time step, which is stored as a float
    float timeStep = eventData[P_TIMESTEP].GetFloat();

    // Move the camera, scale movement with time step
    fpsCounter.Update(timeStep);
    MoveCamera(timeStep);
    UpdateCurrentFpsElement();
}


