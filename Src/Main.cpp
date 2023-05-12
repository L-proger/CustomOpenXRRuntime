#define XR_USE_GRAPHICS_API_D3D11

#include <openxr\openxr.h>

#include <d3d11.h>
#include <openxr\openxr_platform.h>
#include <runtime_interface.hpp>
#include <loader_interfaces.h>

#include <iostream>


#define PRINT_FUNC_CALL std::cout << __FUNCTION__ << std::endl;

extern "C" {

    static XrInstance _lastInstance = {};
    static XrSystemId hmdSystemID = XrSystemId(23);
    static XrSystemId handheldSystemID = XrSystemId(20);

    static XrSession _lastSession = {};
    static XrSpace _lastSpace = {};
    static XrSpace _lastActionSpace = {};


    XrSwapchain _swapchain = {};
    uint32_t _swapchainImageIndex = 0;



    XrResult XRAPI_PTR xrEnumerateDisplayRefreshRatesFB(XrSession session, uint32_t displayRefreshRateCapacityInput, uint32_t* displayRefreshRateCountOutput, float* displayRefreshRates) {
        PRINT_FUNC_CALL;
        if (displayRefreshRateCapacityInput == 0) {
            *displayRefreshRateCountOutput = 1;
            return XrResult::XR_SUCCESS;
        }
        displayRefreshRates[0] = 60.0f;

        return XrResult::XR_SUCCESS;
    }
    XrResult XRAPI_PTR xrGetDisplayRefreshRateFB(XrSession session, float* displayRefreshRate) {
        PRINT_FUNC_CALL;
        *displayRefreshRate = 60;
        return XrResult::XR_SUCCESS;
    }
    XrResult XRAPI_PTR xrRequestDisplayRefreshRateFB(XrSession session, float displayRefreshRate) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrCreateBodyTrackerFB(XrSession session, const XrBodyTrackerCreateInfoFB* createInfo, XrBodyTrackerFB* bodyTracker) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_RUNTIME_FAILURE;
    }
    XrResult XRAPI_PTR xrDestroyBodyTrackerFB(XrBodyTrackerFB bodyTracker) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_RUNTIME_FAILURE;
    }
    XrResult XRAPI_PTR xrLocateBodyJointsFB(XrBodyTrackerFB bodyTracker, const XrBodyJointsLocateInfoFB* locateInfo, XrBodyJointLocationsFB* locations) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_RUNTIME_FAILURE;
    }
    XrResult XRAPI_PTR xrGetBodySkeletonFB(XrBodyTrackerFB bodyTracker, XrBodySkeletonFB* skeleton) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_RUNTIME_FAILURE;
    }




    XrResult XRAPI_PTR xrGetSwapchainStateFB(XrSwapchain swapchain, XrSwapchainStateBaseHeaderFB* state) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_RUNTIME_FAILURE;
    }

    XrResult XRAPI_PTR xrUpdateSwapchainFB(XrSwapchain swapchain, const XrSwapchainStateBaseHeaderFB* state) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_RUNTIME_FAILURE;
    }


    XrResult XRAPI_PTR xrCreateHandTrackerEXT(XrSession session, const XrHandTrackerCreateInfoEXT* createInfo, XrHandTrackerEXT* handTracker) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_RUNTIME_FAILURE;
    }
    XrResult XRAPI_PTR xrDestroyHandTrackerEXT(XrHandTrackerEXT handTracker) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_RUNTIME_FAILURE;
    }
    XrResult XRAPI_PTR xrLocateHandJointsEXT(XrHandTrackerEXT handTracker, const XrHandJointsLocateInfoEXT* locateInfo, XrHandJointLocationsEXT* locations) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_RUNTIME_FAILURE;
    }



    XrResult XRAPI_PTR xrSetInputDeviceActiveEXT(XrSession session, XrPath interactionProfile, XrPath topLevelPath, XrBool32 isActive){
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }
    XrResult XRAPI_PTR xrSetInputDeviceStateBoolEXT(XrSession session, XrPath topLevelPath, XrPath inputSourcePath, XrBool32 state){
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }
    XrResult XRAPI_PTR xrSetInputDeviceStateFloatEXT(XrSession session, XrPath topLevelPath, XrPath inputSourcePath, float state){
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }
    XrResult XRAPI_PTR xrSetInputDeviceStateVector2fEXT(XrSession session, XrPath topLevelPath, XrPath inputSourcePath, XrVector2f state){
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }
    XrResult XRAPI_PTR xrSetInputDeviceLocationEXT(XrSession session, XrPath topLevelPath, XrPath inputSourcePath, XrSpace space, XrPosef pose){
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrSessionInsertDebugUtilsLabelEXT(XrSession session, const XrDebugUtilsLabelEXT* labelInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrSessionEndDebugUtilsLabelRegionEXT(XrSession session) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrSessionBeginDebugUtilsLabelRegionEXT(XrSession session, const XrDebugUtilsLabelEXT* labelInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrSubmitDebugUtilsMessageEXT(XrInstance instance, XrDebugUtilsMessageSeverityFlagsEXT messageSeverity, XrDebugUtilsMessageTypeFlagsEXT messageTypes, const XrDebugUtilsMessengerCallbackDataEXT* callbackData) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrDestroyDebugUtilsMessengerEXT(XrDebugUtilsMessengerEXT messenger) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrCreateDebugUtilsMessengerEXT(XrInstance instance, const XrDebugUtilsMessengerCreateInfoEXT* createInfo, XrDebugUtilsMessengerEXT* messenger) {
        PRINT_FUNC_CALL;
        *messenger = 0;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrSetDebugUtilsObjectNameEXT(XrInstance instance, const XrDebugUtilsObjectNameInfoEXT* nameInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrPerfSettingsSetPerformanceLevelEXT(XrSession session, XrPerfSettingsDomainEXT domain, XrPerfSettingsLevelEXT level) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrConvertWin32PerformanceCounterToTimeKHR(XrInstance instance, const LARGE_INTEGER* performanceCounter, XrTime* time) {
        PRINT_FUNC_CALL;
        *time = performanceCounter->QuadPart;
        return XrResult::XR_SUCCESS;
    }
    XrResult XRAPI_PTR xrConvertTimeToWin32PerformanceCounterKHR(XrInstance instance, XrTime time, LARGE_INTEGER* performanceCounter) {
        PRINT_FUNC_CALL;
        performanceCounter->QuadPart = time;
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrGetVisibilityMaskKHR(XrSession session, XrViewConfigurationType viewConfigurationType, uint32_t viewIndex, XrVisibilityMaskTypeKHR visibilityMaskType, XrVisibilityMaskKHR* visibilityMask) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
    }

    XrResult XRAPI_PTR xrGetD3D11GraphicsRequirementsKHR(XrInstance instance, XrSystemId systemId, XrGraphicsRequirementsD3D11KHR* graphicsRequirements) {
        PRINT_FUNC_CALL;
        graphicsRequirements->minFeatureLevel = D3D_FEATURE_LEVEL_11_0;
        graphicsRequirements->adapterLuid = {};
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrApplyHapticFeedback(XrSession session, const XrHapticActionInfo* hapticActionInfo, const XrHapticBaseHeader* hapticFeedback) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_PATH_INVALID;
    }
    XrResult XRAPI_PTR xrStopHapticFeedback(XrSession session, const XrHapticActionInfo* hapticActionInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_PATH_INVALID;
    }


    XrResult XRAPI_PTR xrGetInputSourceLocalizedName(XrSession session, const XrInputSourceLocalizedNameGetInfo* getInfo, uint32_t bufferCapacityInput, uint32_t* bufferCountOutput, char* buffer) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_PATH_INVALID;
    }

    XrResult XRAPI_PTR xrEnumerateBoundSourcesForAction(XrSession session, const XrBoundSourcesForActionEnumerateInfo* enumerateInfo, uint32_t sourceCapacityInput, uint32_t* sourceCountOutput, XrPath* sources) {
        PRINT_FUNC_CALL;
        return XrResult::XR_ERROR_PATH_INVALID;
    }


    XrResult XRAPI_PTR xrSyncActions(XrSession session, const XrActionsSyncInfo* syncInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrGetActionStatePose(XrSession session, const XrActionStateGetInfo* getInfo, XrActionStatePose* state) {
        PRINT_FUNC_CALL;
        state[0].isActive = XR_FALSE;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrGetActionStateVector2f(XrSession session, const XrActionStateGetInfo* getInfo, XrActionStateVector2f* state) {
        PRINT_FUNC_CALL;
        state[0].changedSinceLastSync = XR_FALSE;
        state[0].currentState = {0.0f, 0.0f};
        state[0].isActive = XR_FALSE;
        state[0].lastChangeTime = 0;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrGetActionStateFloat(XrSession session, const XrActionStateGetInfo* getInfo, XrActionStateFloat* state) {
        PRINT_FUNC_CALL;
        state[0].changedSinceLastSync = XR_FALSE;
        state[0].currentState = 0.0f;
        state[0].isActive = XR_FALSE;
        state[0].lastChangeTime = 0;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrGetActionStateBoolean(XrSession session, const XrActionStateGetInfo* getInfo, XrActionStateBoolean* state) {
        PRINT_FUNC_CALL;
        state[0].changedSinceLastSync = XR_FALSE;
        state[0].currentState = XR_FALSE;
        state[0].isActive = XR_FALSE;
        state[0].lastChangeTime = 0;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrGetCurrentInteractionProfile(XrSession session, XrPath topLevelUserPath, XrInteractionProfileState* interactionProfile) {
        PRINT_FUNC_CALL;
        interactionProfile->interactionProfile = 0;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrAttachSessionActionSets(XrSession session, const XrSessionActionSetsAttachInfo* attachInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrSuggestInteractionProfileBindings(XrInstance instance, const XrInteractionProfileSuggestedBinding* suggestedBindings) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrDestroyAction(XrAction action) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrCreateAction(XrActionSet actionSet, const XrActionCreateInfo* createInfo, XrAction* action) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrDestroyActionSet(XrActionSet actionSet) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrCreateActionSet(XrInstance instance, const XrActionSetCreateInfo* createInfo, XrActionSet* actionSet) {
        PRINT_FUNC_CALL;
        *actionSet = XrActionSet(11);
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrPathToString(XrInstance instance, XrPath path, uint32_t bufferCapacityInput, uint32_t* bufferCountOutput, char* buffer) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrStringToPath(XrInstance instance, const char* pathString, XrPath* path) {
        PRINT_FUNC_CALL;
        *path = XR_NULL_PATH;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrLocateViews(XrSession session, const XrViewLocateInfo* viewLocateInfo, XrViewState* viewState, uint32_t viewCapacityInput, uint32_t* viewCountOutput, XrView* views) {
        PRINT_FUNC_CALL;
        if (viewCapacityInput == 0) {
            *viewCountOutput = 1;
            return XrResult::XR_SUCCESS;
        }

        *viewCountOutput = 1;

        views[0].fov = {45.0f, 45.0f, 45.0f, 45.0f };
        views[0].pose.orientation = { 1.0f, 0.0f, 0.0f, 0.0f };
        views[0].pose.position = { 0.0f, 1.0f, 3.0f };

        viewState->viewStateFlags = XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT | XR_VIEW_STATE_ORIENTATION_TRACKED_BIT | XR_VIEW_STATE_POSITION_TRACKED_BIT;

        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrEndFrame(XrSession session, const XrFrameEndInfo* frameEndInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrBeginFrame(XrSession session, const XrFrameBeginInfo* frameBeginInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrWaitFrame(XrSession session, const XrFrameWaitInfo* frameWaitInfo, XrFrameState* frameState) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrRequestExitSession(XrSession session){
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrEndSession(XrSession session) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrBeginSession(XrSession session, const XrSessionBeginInfo* beginInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrReleaseSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageReleaseInfo* releaseInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrWaitSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageWaitInfo* waitInfo) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrAcquireSwapchainImage(XrSwapchain swapchain, const XrSwapchainImageAcquireInfo* acquireInfo, uint32_t* index) {
        PRINT_FUNC_CALL;
        *index = _swapchainImageIndex;
        _swapchainImageIndex = (_swapchainImageIndex + 1) % 2;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrEnumerateSwapchainImages(XrSwapchain swapchain, uint32_t imageCapacityInput, uint32_t* imageCountOutput, XrSwapchainImageBaseHeader* images) {
        PRINT_FUNC_CALL;
        if (imageCapacityInput == 0) {
            *imageCountOutput = 2;
            return XrResult::XR_SUCCESS;
        }

        if (imageCapacityInput != 2) {
            return XrResult::XR_ERROR_SIZE_INSUFFICIENT;
        }

        if (images[0].type != XrStructureType::XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR) {
            return XrResult::XR_ERROR_VALIDATION_FAILURE;
        }
        if (images[1].type != XrStructureType::XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR) {
            return XrResult::XR_ERROR_VALIDATION_FAILURE;
        }

        auto ptr0 = reinterpret_cast<XrSwapchainImageD3D11KHR*>(&images[0]);
        ptr0->texture = nullptr;

        auto ptr1 = reinterpret_cast<XrSwapchainImageD3D11KHR*>(&images[1]);
        ptr1->texture = nullptr;

        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrDestroySwapchain(XrSwapchain swapchain) {
        PRINT_FUNC_CALL;
        _swapchain = {};
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrCreateSwapchain(XrSession session, const XrSwapchainCreateInfo* createInfo, XrSwapchain* swapchain) {
        PRINT_FUNC_CALL;
        *swapchain = _swapchain;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrEnumerateSwapchainFormats(XrSession session, uint32_t formatCapacityInput, uint32_t* formatCountOutput, int64_t* formats) {
        PRINT_FUNC_CALL;
        if (formatCapacityInput == 0) {
            *formatCountOutput = 1;
            return XrResult::XR_SUCCESS;
        }
        *formatCountOutput = 1;
        formats[0] = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrEnumerateViewConfigurationViews(XrInstance instance, XrSystemId systemId, XrViewConfigurationType viewConfigurationType, uint32_t viewCapacityInput, uint32_t* viewCountOutput, XrViewConfigurationView* views) {
        PRINT_FUNC_CALL;
        if (viewCapacityInput == 0) {
            *viewCountOutput = 2;
            return XrResult::XR_SUCCESS;
        }

        views[0].maxImageRectWidth = 1920;
        views[0].maxImageRectHeight = 1080;
        views[0].maxSwapchainSampleCount = 1;
        views[0].recommendedImageRectWidth = 1920;
        views[0].recommendedImageRectHeight = 1080;
        views[0].recommendedSwapchainSampleCount = 1;
        *viewCountOutput = 1;
        if (viewCapacityInput == 2) {
            views[1].maxImageRectWidth = 1920;
            views[1].maxImageRectHeight = 1080;
            views[1].maxSwapchainSampleCount = 1;
            views[1].recommendedImageRectWidth = 1920;
            views[1].recommendedImageRectHeight = 1080;
            views[1].recommendedSwapchainSampleCount = 1;
            *viewCountOutput = 2;
        }
        return XrResult::XR_SUCCESS;

    }

    XrResult XRAPI_PTR xrGetViewConfigurationProperties(XrInstance instance, XrSystemId systemId, XrViewConfigurationType viewConfigurationType, XrViewConfigurationProperties* configurationProperties) {
        PRINT_FUNC_CALL;
        configurationProperties->viewConfigurationType = viewConfigurationType;
        configurationProperties->fovMutable = false;
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrEnumerateViewConfigurations(XrInstance instance, XrSystemId systemId, uint32_t viewConfigurationTypeCapacityInput, uint32_t* viewConfigurationTypeCountOutput, XrViewConfigurationType* viewConfigurationTypes) {
        PRINT_FUNC_CALL;
        if (viewConfigurationTypeCapacityInput == 0) {
            *viewConfigurationTypeCountOutput = 2;
            return XrResult::XR_SUCCESS;
        }
        viewConfigurationTypes[0] = XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        if (viewConfigurationTypeCapacityInput > 1) {
            viewConfigurationTypes[1] = XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO;
        }

        *viewConfigurationTypeCountOutput = viewConfigurationTypeCapacityInput;
        
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrDestroySpace(XrSpace space) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrLocateSpace(XrSpace space, XrSpace baseSpace, XrTime time, XrSpaceLocation* location) {
        PRINT_FUNC_CALL;
        location->locationFlags = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT | XR_SPACE_LOCATION_POSITION_TRACKED_BIT;
        location->pose.position = { 0, 1, 3 };
        location->pose.orientation = { 1.0f, 0.0f, 0.0f, 0.0f };
        location->next = nullptr;
        location->type = XR_TYPE_SPACE_LOCATION;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrCreateActionSpace(XrSession session, const XrActionSpaceCreateInfo* createInfo, XrSpace* space) {
        PRINT_FUNC_CALL;
        _lastActionSpace = XrSpace(77);
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrGetReferenceSpaceBoundsRect(XrSession session, XrReferenceSpaceType referenceSpaceType, XrExtent2Df* bounds) {
        PRINT_FUNC_CALL;
        if (referenceSpaceType != XrReferenceSpaceType::XR_REFERENCE_SPACE_TYPE_VIEW) {
            return XrResult::XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
        }
        bounds->width = 10.0f;
        bounds->height = 10.0f;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrCreateReferenceSpace(XrSession session, const XrReferenceSpaceCreateInfo* createInfo, XrSpace* space) {
        PRINT_FUNC_CALL;
        if (createInfo->referenceSpaceType != XrReferenceSpaceType::XR_REFERENCE_SPACE_TYPE_VIEW) {
            return XrResult::XR_ERROR_REFERENCE_SPACE_UNSUPPORTED;
        }
        
        *space = _lastSpace;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrEnumerateReferenceSpaces(XrSession session, uint32_t spaceCapacityInput, uint32_t* spaceCountOutput, XrReferenceSpaceType* spaces) {
        PRINT_FUNC_CALL;
        *spaceCountOutput = 1;
        if (spaceCapacityInput == 0) {
            return XrResult::XR_SUCCESS;
        }
        spaces[0] = XrReferenceSpaceType::XR_REFERENCE_SPACE_TYPE_VIEW;
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrDestroySession(XrSession session) {
        PRINT_FUNC_CALL;
        _lastSession = {};
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrCreateSession(XrInstance instance, const XrSessionCreateInfo* createInfo, XrSession* session) {
        PRINT_FUNC_CALL;
        if (createInfo->next != nullptr) {
            auto d3d11Params = reinterpret_cast<const XrGraphicsBindingD3D11KHR*>(createInfo->next);
            if (d3d11Params->type == XR_TYPE_GRAPHICS_BINDING_D3D11_KHR) {
                if (d3d11Params->device == nullptr) {
                    return XrResult::XR_ERROR_GRAPHICS_DEVICE_INVALID;
                }
                else {
                    std::cout << "Received D3D11 device" << std::endl;
                }
            }
        }

        _lastSession = XrSession(234);
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrEnumerateEnvironmentBlendModes(XrInstance instance, XrSystemId systemId, XrViewConfigurationType viewConfigurationType, uint32_t environmentBlendModeCapacityInput, uint32_t* environmentBlendModeCountOutput, XrEnvironmentBlendMode* environmentBlendModes) {
        PRINT_FUNC_CALL;
        if ((viewConfigurationType != XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO) && (viewConfigurationType != XrViewConfigurationType::XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO)) {
            return XrResult::XR_ERROR_VIEW_CONFIGURATION_TYPE_UNSUPPORTED;
        }
        
        *environmentBlendModeCountOutput = 1;
        if (environmentBlendModeCapacityInput == 0) {
            return XrResult::XR_SUCCESS;
        }
        environmentBlendModes[0] = XrEnvironmentBlendMode::XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        return XrResult::XR_SUCCESS;
    }


    XrResult XRAPI_PTR xrGetSystemProperties(XrInstance instance, XrSystemId systemId, XrSystemProperties* properties) {
        PRINT_FUNC_CALL;
        if (instance != _lastInstance) {
            return XrResult::XR_ERROR_HANDLE_INVALID;
        }

        properties->systemId = systemId;
        properties->vendorId = 0xA0BB;
        properties->trackingProperties.orientationTracking = XR_TRUE;
        properties->trackingProperties.positionTracking = XR_TRUE;
        properties->graphicsProperties.maxLayerCount = XR_MIN_COMPOSITION_LAYERS_SUPPORTED;
        properties->graphicsProperties.maxSwapchainImageWidth = 3840;
        properties->graphicsProperties.maxSwapchainImageHeight = 2160;

        if (systemId == hmdSystemID) {
            strcpy(properties->systemName, "customhmd");
        }if (systemId == handheldSystemID) {
            strcpy(properties->systemName, "customhandheld");
        }
        else {
            return XrResult::XR_ERROR_SYSTEM_INVALID;
        }
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrGetSystem(XrInstance instance, const XrSystemGetInfo* getInfo, XrSystemId* systemId){
        PRINT_FUNC_CALL;

        if (getInfo->formFactor == XrFormFactor::XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY) {
            *systemId = hmdSystemID;
            return XrResult::XR_SUCCESS;
        }else if (getInfo->formFactor == XrFormFactor::XR_FORM_FACTOR_HANDHELD_DISPLAY) {
            *systemId = handheldSystemID;
            return XrResult::XR_SUCCESS;
        }

        return XrResult::XR_ERROR_FORM_FACTOR_UNSUPPORTED;
    }

    XrResult XRAPI_PTR xrStructureTypeToString(XrInstance instance, XrStructureType value, char buffer[XR_MAX_STRUCTURE_NAME_SIZE]) {
        PRINT_FUNC_CALL;
        strcpy(buffer, "NOT_IMPLEMENTED");
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrResultToString(XrInstance instance, XrResult value, char buffer[XR_MAX_RESULT_STRING_SIZE]) {
        PRINT_FUNC_CALL;
        strcpy(buffer, "NOT_IMPLEMENTED");
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrPollEvent(XrInstance instance, XrEventDataBuffer* eventData) {
        PRINT_FUNC_CALL;
        return XrResult::XR_EVENT_UNAVAILABLE;
    }

    XrResult XRAPI_PTR xrGetInstanceProperties(XrInstance instance, XrInstanceProperties* instanceProperties) {
        PRINT_FUNC_CALL;
        strcpy(instanceProperties->runtimeName, "customxr");
        instanceProperties->runtimeVersion = XR_MAKE_VERSION(0, 1, 0);
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrDestroyInstance(XrInstance instance) {
        PRINT_FUNC_CALL;
        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrCreateInstance(const XrInstanceCreateInfo* createInfo, XrInstance* instance) {
        PRINT_FUNC_CALL;
        _lastInstance = (XrInstance)123;
        *instance = _lastInstance;


        return XrResult::XR_SUCCESS;
    }

    XrResult XRAPI_PTR xrEnumerateInstanceExtensionProperties(const char* layerName, uint32_t propertyCapacityInput, uint32_t* propertyCountOutput, XrExtensionProperties* properties) {
        PRINT_FUNC_CALL;
        if (propertyCapacityInput == 0) {
            *propertyCountOutput = 1;
            return XrResult::XR_SUCCESS;
        }
        strcpy(properties[0].extensionName, XR_KHR_D3D11_ENABLE_EXTENSION_NAME);
        properties[0].extensionVersion = 9;
        properties->next = nullptr;
        
        *propertyCountOutput = 1;
        return XrResult::XR_SUCCESS;
    }


#define RETURN_PROC_ADDR(procName)    \
    if (strcmp(name, #procName) == 0) { \
        *function = (PFN_xrVoidFunction)&procName; \
        return XrResult::XR_SUCCESS; \
    }

#define RETURN_NULL_PROC_ADDR(procName)    \
    if (strcmp(name, #procName) == 0) { \
        *function = (PFN_xrVoidFunction)nullptr; \
        return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED; \
    }

    bool hasEnding(std::string const& fullString, std::string const& ending) {
        if (fullString.length() >= ending.length()) {
            return (0 == fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
        }
        else {
            return false;
        }
    }

    XrResult XRAPI_PTR xrGetInstanceProcAddr(XrInstance instance, const char* name, PFN_xrVoidFunction* function) {
        RETURN_PROC_ADDR(xrEnumerateInstanceExtensionProperties);
        RETURN_PROC_ADDR(xrCreateInstance);
        RETURN_PROC_ADDR(xrDestroyInstance);
        RETURN_PROC_ADDR(xrGetInstanceProperties);
        RETURN_PROC_ADDR(xrPollEvent);
        RETURN_PROC_ADDR(xrResultToString);
        RETURN_PROC_ADDR(xrStructureTypeToString);
        RETURN_PROC_ADDR(xrGetSystem);
        RETURN_PROC_ADDR(xrGetSystemProperties);
        RETURN_PROC_ADDR(xrEnumerateEnvironmentBlendModes);
        RETURN_PROC_ADDR(xrCreateSession);
        RETURN_PROC_ADDR(xrDestroySession);
        RETURN_PROC_ADDR(xrEnumerateReferenceSpaces);
        RETURN_PROC_ADDR(xrCreateReferenceSpace);
        RETURN_PROC_ADDR(xrGetReferenceSpaceBoundsRect);
        RETURN_PROC_ADDR(xrCreateActionSpace);
        RETURN_PROC_ADDR(xrLocateSpace);
        RETURN_PROC_ADDR(xrDestroySpace);
        RETURN_PROC_ADDR(xrEnumerateViewConfigurations);
        RETURN_PROC_ADDR(xrGetViewConfigurationProperties);
        RETURN_PROC_ADDR(xrEnumerateViewConfigurationViews);
        RETURN_PROC_ADDR(xrEnumerateSwapchainFormats);
        RETURN_PROC_ADDR(xrCreateSwapchain);
        RETURN_PROC_ADDR(xrDestroySwapchain);
        RETURN_PROC_ADDR(xrEnumerateSwapchainImages);
        RETURN_PROC_ADDR(xrAcquireSwapchainImage);
        RETURN_PROC_ADDR(xrWaitSwapchainImage);
        RETURN_PROC_ADDR(xrReleaseSwapchainImage);
        RETURN_PROC_ADDR(xrBeginSession);
        RETURN_PROC_ADDR(xrEndSession);
        RETURN_PROC_ADDR(xrRequestExitSession);
        RETURN_PROC_ADDR(xrWaitFrame);
        RETURN_PROC_ADDR(xrBeginFrame);
        RETURN_PROC_ADDR(xrEndFrame);
        RETURN_PROC_ADDR(xrLocateViews);
        RETURN_PROC_ADDR(xrStringToPath);
        RETURN_PROC_ADDR(xrPathToString);
        RETURN_PROC_ADDR(xrCreateActionSet);
        RETURN_PROC_ADDR(xrDestroyActionSet);
        RETURN_PROC_ADDR(xrCreateAction);
        RETURN_PROC_ADDR(xrDestroyAction);
        RETURN_PROC_ADDR(xrSuggestInteractionProfileBindings);
        RETURN_PROC_ADDR(xrAttachSessionActionSets);
        RETURN_PROC_ADDR(xrGetCurrentInteractionProfile);
        RETURN_PROC_ADDR(xrGetActionStateBoolean);
        RETURN_PROC_ADDR(xrGetActionStateFloat);
        RETURN_PROC_ADDR(xrGetActionStateVector2f);
        RETURN_PROC_ADDR(xrGetActionStatePose);
        RETURN_PROC_ADDR(xrSyncActions);
        RETURN_PROC_ADDR(xrEnumerateBoundSourcesForAction);
        RETURN_PROC_ADDR(xrGetInputSourceLocalizedName);
        RETURN_PROC_ADDR(xrStopHapticFeedback);
        RETURN_PROC_ADDR(xrApplyHapticFeedback);
        RETURN_NULL_PROC_ADDR(xrGetOpenGLGraphicsRequirementsKHR)
        RETURN_NULL_PROC_ADDR(xrGetVulkanInstanceExtensionsKHR)
        RETURN_NULL_PROC_ADDR(xrGetVulkanDeviceExtensionsKHR)
        RETURN_NULL_PROC_ADDR(xrGetVulkanGraphicsDeviceKHR)
        RETURN_NULL_PROC_ADDR(xrGetVulkanGraphicsRequirementsKHR)

        RETURN_NULL_PROC_ADDR(xrConvertTimespecTimeToTimeKHR)
        RETURN_NULL_PROC_ADDR(xrConvertTimeToTimespecTimeKHR)

        RETURN_NULL_PROC_ADDR(xrGetD3D12GraphicsRequirementsKHR)
        RETURN_NULL_PROC_ADDR(xrCreateVulkanInstanceKHR)
        RETURN_NULL_PROC_ADDR(xrCreateVulkanDeviceKHR)
        RETURN_NULL_PROC_ADDR(xrGetVulkanGraphicsDevice2KHR)
        RETURN_NULL_PROC_ADDR(xrGetVulkanGraphicsRequirements2KHR)
        RETURN_NULL_PROC_ADDR(xrThermalGetTemperatureTrendEXT)
        RETURN_NULL_PROC_ADDR(xrCreateSpatialAnchorMSFT)
        RETURN_NULL_PROC_ADDR(xrCreateSpatialAnchorSpaceMSFT)
        RETURN_NULL_PROC_ADDR(xrDestroySpatialAnchorMSFT)
        RETURN_NULL_PROC_ADDR(xrCreateSpatialGraphNodeSpaceMSFT)
        RETURN_NULL_PROC_ADDR(xrTryCreateSpatialGraphStaticNodeBindingMSFT)
        RETURN_NULL_PROC_ADDR(xrDestroySpatialGraphNodeBindingMSFT)
        RETURN_NULL_PROC_ADDR(xrGetSpatialGraphNodeBindingPropertiesMSFT)
        RETURN_NULL_PROC_ADDR(xrCreateHandMeshSpaceMSFT)
        RETURN_NULL_PROC_ADDR(xrUpdateHandMeshMSFT)
        RETURN_NULL_PROC_ADDR(xrGetControllerModelKeyMSFT)
        RETURN_NULL_PROC_ADDR(xrLoadControllerModelMSFT)
        RETURN_NULL_PROC_ADDR(xrGetControllerModelPropertiesMSFT)
        RETURN_NULL_PROC_ADDR(xrGetControllerModelStateMSFT)
        RETURN_NULL_PROC_ADDR(xrCreateSpatialAnchorFromPerceptionAnchorMSFT)
        RETURN_NULL_PROC_ADDR(xrTryGetPerceptionAnchorFromSpatialAnchorMSFT)
        RETURN_NULL_PROC_ADDR(xrEnumerateReprojectionModesMSFT)
        RETURN_NULL_PROC_ADDR(xrEnumerateSceneComputeFeaturesMSFT)
        RETURN_NULL_PROC_ADDR(xrCreateSceneObserverMSFT)
        RETURN_NULL_PROC_ADDR(xrDestroySceneObserverMSFT) 
        RETURN_NULL_PROC_ADDR(xrCreateSceneMSFT)
        RETURN_NULL_PROC_ADDR(xrDestroySceneMSFT)
        RETURN_NULL_PROC_ADDR(xrComputeNewSceneMSFT)
        RETURN_NULL_PROC_ADDR(xrGetSceneComputeStateMSFT)

            

            
            
        RETURN_PROC_ADDR(xrGetD3D11GraphicsRequirementsKHR);
        RETURN_PROC_ADDR(xrGetVisibilityMaskKHR);
        RETURN_PROC_ADDR(xrConvertWin32PerformanceCounterToTimeKHR);
        RETURN_PROC_ADDR(xrConvertTimeToWin32PerformanceCounterKHR);
        RETURN_PROC_ADDR(xrPerfSettingsSetPerformanceLevelEXT);
        RETURN_PROC_ADDR(xrSetDebugUtilsObjectNameEXT);
        RETURN_PROC_ADDR(xrCreateDebugUtilsMessengerEXT);
        RETURN_PROC_ADDR(xrDestroyDebugUtilsMessengerEXT);
        RETURN_PROC_ADDR(xrSubmitDebugUtilsMessageEXT);
        RETURN_PROC_ADDR(xrSessionBeginDebugUtilsLabelRegionEXT);
        RETURN_PROC_ADDR(xrSessionEndDebugUtilsLabelRegionEXT);
        RETURN_PROC_ADDR(xrSessionInsertDebugUtilsLabelEXT);


        RETURN_PROC_ADDR(xrSetInputDeviceActiveEXT);
        RETURN_PROC_ADDR(xrSetInputDeviceStateBoolEXT);
        RETURN_PROC_ADDR(xrSetInputDeviceStateFloatEXT);
        RETURN_PROC_ADDR(xrSetInputDeviceStateVector2fEXT);
        RETURN_PROC_ADDR(xrSetInputDeviceLocationEXT);

        RETURN_PROC_ADDR(xrCreateHandTrackerEXT);
        RETURN_PROC_ADDR(xrDestroyHandTrackerEXT);
        RETURN_PROC_ADDR(xrLocateHandJointsEXT);


        RETURN_PROC_ADDR(xrUpdateSwapchainFB);
        RETURN_PROC_ADDR(xrGetSwapchainStateFB);

        
        RETURN_PROC_ADDR(xrCreateBodyTrackerFB);
        RETURN_PROC_ADDR(xrDestroyBodyTrackerFB);
        RETURN_PROC_ADDR(xrLocateBodyJointsFB);
        RETURN_PROC_ADDR(xrGetBodySkeletonFB);

        RETURN_NULL_PROC_ADDR(xrGetSceneComponentsMSFT);
        RETURN_NULL_PROC_ADDR(xrLocateSceneComponentsMSFT);
        RETURN_NULL_PROC_ADDR(xrGetSceneMeshBuffersMSFT);
        RETURN_NULL_PROC_ADDR(xrDeserializeSceneMSFT);
        RETURN_NULL_PROC_ADDR(xrGetSerializedSceneFragmentDataMSFT);
        RETURN_NULL_PROC_ADDR(xrEnumerateViveTrackerPathsHTCX);

        

        RETURN_PROC_ADDR(xrEnumerateDisplayRefreshRatesFB);
        RETURN_PROC_ADDR(xrGetDisplayRefreshRateFB);
        RETURN_PROC_ADDR(xrRequestDisplayRefreshRateFB);
        
        RETURN_NULL_PROC_ADDR(xrCreateFacialTrackerHTC);
        RETURN_NULL_PROC_ADDR(xrDestroyFacialTrackerHTC);
        RETURN_NULL_PROC_ADDR(xrGetFacialExpressionsHTC);


        RETURN_NULL_PROC_ADDR(xrEnumerateColorSpacesFB);
        RETURN_NULL_PROC_ADDR(xrSetColorSpaceFB);
        RETURN_NULL_PROC_ADDR(xrGetHandMeshFB);
        RETURN_NULL_PROC_ADDR(xrCreateSpatialAnchorFB);
        RETURN_NULL_PROC_ADDR(xrGetSpaceUuidFB);
        RETURN_NULL_PROC_ADDR(xrEnumerateSpaceSupportedComponentsFB);
        RETURN_NULL_PROC_ADDR(xrSetSpaceComponentStatusFB);
        RETURN_NULL_PROC_ADDR(xrGetSpaceComponentStatusFB);
        RETURN_NULL_PROC_ADDR(xrCreateFoveationProfileFB);
        RETURN_NULL_PROC_ADDR(xrDestroyFoveationProfileFB);
        RETURN_NULL_PROC_ADDR(xrQuerySystemTrackedKeyboardFB);
        RETURN_NULL_PROC_ADDR(xrCreateKeyboardSpaceFB);
        RETURN_NULL_PROC_ADDR(xrCreateTriangleMeshFB);
        RETURN_NULL_PROC_ADDR(xrDestroyTriangleMeshFB);
        RETURN_NULL_PROC_ADDR(xrTriangleMeshGetVertexBufferFB);
        RETURN_NULL_PROC_ADDR(xrTriangleMeshGetIndexBufferFB);
        RETURN_NULL_PROC_ADDR(xrTriangleMeshBeginUpdateFB);
        RETURN_NULL_PROC_ADDR(xrTriangleMeshEndUpdateFB);
        RETURN_NULL_PROC_ADDR(xrTriangleMeshBeginVertexBufferUpdateFB);
        RETURN_NULL_PROC_ADDR(xrTriangleMeshEndVertexBufferUpdateFB);
        RETURN_NULL_PROC_ADDR(xrCreatePassthroughFB);
        RETURN_NULL_PROC_ADDR(xrDestroyPassthroughFB);
        RETURN_NULL_PROC_ADDR(xrPassthroughStartFB);
        RETURN_NULL_PROC_ADDR(xrPassthroughPauseFB);
        RETURN_NULL_PROC_ADDR(xrCreatePassthroughLayerFB);
        RETURN_NULL_PROC_ADDR(xrDestroyPassthroughLayerFB);
        RETURN_NULL_PROC_ADDR(xrPassthroughLayerPauseFB);
        RETURN_NULL_PROC_ADDR(xrPassthroughLayerResumeFB);
        RETURN_NULL_PROC_ADDR(xrPassthroughLayerSetStyleFB);
        RETURN_NULL_PROC_ADDR(xrCreateGeometryInstanceFB);
        RETURN_NULL_PROC_ADDR(xrDestroyGeometryInstanceFB);
        RETURN_NULL_PROC_ADDR(xrGeometryInstanceSetTransformFB);
        RETURN_NULL_PROC_ADDR(xrEnumerateRenderModelPathsFB);
        RETURN_NULL_PROC_ADDR(xrGetRenderModelPropertiesFB);
        RETURN_NULL_PROC_ADDR(xrLoadRenderModelFB);


        RETURN_NULL_PROC_ADDR(xrSetEnvironmentDepthEstimationVARJO);
        RETURN_NULL_PROC_ADDR(xrSetMarkerTrackingVARJO);
        RETURN_NULL_PROC_ADDR(xrSetMarkerTrackingTimeoutVARJO);
        RETURN_NULL_PROC_ADDR(xrSetMarkerTrackingPredictionVARJO);
        RETURN_NULL_PROC_ADDR(xrGetMarkerSizeVARJO);
        RETURN_NULL_PROC_ADDR(xrCreateMarkerSpaceVARJO);
        RETURN_NULL_PROC_ADDR(xrSetViewOffsetVARJO);
        RETURN_NULL_PROC_ADDR(xrCreateSpatialAnchorStoreConnectionMSFT);
        RETURN_NULL_PROC_ADDR(xrDestroySpatialAnchorStoreConnectionMSFT);
        RETURN_NULL_PROC_ADDR(xrPersistSpatialAnchorMSFT);
        RETURN_NULL_PROC_ADDR(xrEnumeratePersistedSpatialAnchorNamesMSFT);
        RETURN_NULL_PROC_ADDR(xrCreateSpatialAnchorFromPersistedNameMSFT);
        RETURN_NULL_PROC_ADDR(xrUnpersistSpatialAnchorMSFT);
        RETURN_NULL_PROC_ADDR(xrClearSpatialAnchorStoreMSFT);
        RETURN_NULL_PROC_ADDR(xrQuerySpacesFB);
        RETURN_NULL_PROC_ADDR(xrRetrieveSpaceQueryResultsFB);
        RETURN_NULL_PROC_ADDR(xrSaveSpaceFB);
        RETURN_NULL_PROC_ADDR(xrEraseSpaceFB);



        RETURN_NULL_PROC_ADDR(xrGetAudioOutputDeviceGuidOculus);
        RETURN_NULL_PROC_ADDR(xrGetAudioInputDeviceGuidOculus);
        RETURN_NULL_PROC_ADDR(xrShareSpacesFB);
        RETURN_NULL_PROC_ADDR(xrGetSpaceBoundingBox2DFB);
        RETURN_NULL_PROC_ADDR(xrGetSpaceBoundingBox3DFB);
        RETURN_NULL_PROC_ADDR(xrGetSpaceSemanticLabelsFB);
        RETURN_NULL_PROC_ADDR(xrGetSpaceBoundary2DFB);
        RETURN_NULL_PROC_ADDR(xrGetSpaceRoomLayoutFB);


        RETURN_NULL_PROC_ADDR(xrSetDigitalLensControlALMALENCE);
        RETURN_NULL_PROC_ADDR(xrRequestSceneCaptureFB);
        RETURN_NULL_PROC_ADDR(xrGetSpaceContainerFB);

        RETURN_NULL_PROC_ADDR(xrGetFoveationEyeTrackedStateMETA);
        RETURN_NULL_PROC_ADDR(xrCreateFaceTrackerFB);
        RETURN_NULL_PROC_ADDR(xrDestroyFaceTrackerFB);
        RETURN_NULL_PROC_ADDR(xrGetFaceExpressionWeightsFB);
        RETURN_NULL_PROC_ADDR(xrCreateEyeTrackerFB);
        RETURN_NULL_PROC_ADDR(xrDestroyEyeTrackerFB);
        RETURN_NULL_PROC_ADDR(xrGetEyeGazesFB);
        RETURN_NULL_PROC_ADDR(xrPassthroughLayerSetKeyboardHandsIntensityFB);
        RETURN_NULL_PROC_ADDR(xrGetDeviceSampleRateFB);

        
        RETURN_NULL_PROC_ADDR(xrEnumerateExternalCamerasOCULUS);
        RETURN_NULL_PROC_ADDR(xrEnumeratePerformanceMetricsCounterPathsMETA);
        RETURN_NULL_PROC_ADDR(xrSetPerformanceMetricsStateMETA);
        RETURN_NULL_PROC_ADDR(xrGetPerformanceMetricsStateMETA);
        RETURN_NULL_PROC_ADDR(xrQueryPerformanceMetricsCounterMETA);
        RETURN_NULL_PROC_ADDR(xrSaveSpaceListFB);
        RETURN_NULL_PROC_ADDR(xrCreateSpaceUserFB);
        RETURN_NULL_PROC_ADDR(xrGetSpaceUserIdFB);
        RETURN_NULL_PROC_ADDR(xrDestroySpaceUserFB);


        RETURN_NULL_PROC_ADDR(xrSetTrackingOptimizationSettingsHintQCOM);
        RETURN_NULL_PROC_ADDR(xrCreatePassthroughHTC);
        RETURN_NULL_PROC_ADDR(xrDestroyPassthroughHTC);
        RETURN_NULL_PROC_ADDR(xrApplyFoveationHTC);


        RETURN_NULL_PROC_ADDR(xrApplyForceFeedbackCurlMNDX);




        if (hasEnding(name, "MSFT")) {
            std::cout << "MSFT function: " << name << std::endl;
            return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
        }
        if (hasEnding(name, "HTCX") || hasEnding(name, "HTC")) {
            std::cout << "HTC function: " << name << std::endl;
            return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
        }

        if (hasEnding(name, "VARJO")) {
            std::cout << "VARJO function: " << name << std::endl;
            return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
        }

        //
        if (hasEnding(name, "FB")) {
            std::cout << "FB function: " << name << std::endl;
            return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
        }

        if (hasEnding(name, "Oculus")) {
            std::cout << "Oculus function: " << name << std::endl;
            return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
        }

        if (hasEnding(name, "ALMALENCE")) {
            std::cout << "ALMALENCE function: " << name << std::endl;
            return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
        }

        if (hasEnding(name, "META")) {
            std::cout << "META function: " << name << std::endl;
            return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
        }
        if (hasEnding(name, "OCULUS")) {
            std::cout << "OCULUS function: " << name << std::endl;
            return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
        }

        if (hasEnding(name, "QCOM")) {
            std::cout << "QCOM function: " << name << std::endl;
            return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
        }
        if (hasEnding(name, "MNDX")) {
            std::cout << "MNDX function: " << name << std::endl;
            return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
        }


        
        

        *function = nullptr;
        return XrResult::XR_ERROR_FUNCTION_UNSUPPORTED;
    }


    __declspec(dllexport) XrResult XRAPI_PTR xrNegotiateLoaderRuntimeInterface(const XrNegotiateLoaderInfo *loaderInfo, XrNegotiateRuntimeRequest *runtimeRequest) {

        runtimeRequest->runtimeInterfaceVersion = loaderInfo->maxInterfaceVersion;
        runtimeRequest->runtimeApiVersion = XR_CURRENT_API_VERSION;
        runtimeRequest->getInstanceProcAddr = &xrGetInstanceProcAddr;

        return XrResult::XR_SUCCESS;
    }
}