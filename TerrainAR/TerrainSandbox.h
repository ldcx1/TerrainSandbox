// Copyright (c) 2008-2023 the Urho3D project
// License: MIT

#pragma once

#include <Urho3D/Math/Plane.h>
#include <Urho3D/Resource/Image.h>

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

#include "Sample.h"

namespace Urho3D
{
    class Node;
    class Scene;
    class Image;
}

/// Water example.
/// This sample demonstrates:
///     - Creating a large plane to represent a water body for rendering
///     - Setting up a second camera to render reflections on the water surface
class Water : public Sample
{
    URHO3D_OBJECT(Water, Sample);

public:
    /// Construct.
    explicit Water(Context* context);

    /// Setup after engine initialization and before running the main loop.
    void Start() override;

private:
    /// Construct the scene content.
    void CreateScene();
    /// Construct an instruction text to the UI.
    void CreateInstructions();
    /// Set up a viewport for displaying the scene.
    void SetupViewport();
    /// Subscribe to the logic update event.
    void SubscribeToEvents();
    /// Read input and moves the camera.
    void MoveCamera(float timeStep);
    /// Handle the logic update event.
    void HandleUpdate(StringHash eventType, VariantMap& eventData);

    void CreateCurrentFpsUiElement();

    void UpdateCurrentFpsElement();
};
