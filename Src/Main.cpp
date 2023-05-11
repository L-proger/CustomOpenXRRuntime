#include <openxr\openxr.h>
#include <runtime_interface.hpp>
#include <loader_interfaces.h>

extern "C" {
    __declspec(dllexport) XrResult XRAPI_PTR xrNegotiateLoaderRuntimeInterface(const XrNegotiateLoaderInfo *loaderInfo, XrNegotiateRuntimeRequest *runtimeRequest) {
        return XrResult::XR_SUCCESS;
    }
}