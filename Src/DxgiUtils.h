#pragma once

#include <dxgi.h>
#include <vector>
#include <stdexcept>

#include <wrl/client.h>
#pragma comment(lib, "DXGI.lib")

class DxgiUtils {
public:
    DxgiUtils() {
        if (CreateDXGIFactory(__uuidof(IDXGIFactory), &_dxgiFactory) != S_OK) {
            throw std::runtime_error("Failed to create DXGI factory");
        }
    }

    std::vector<Microsoft::WRL::ComPtr<IDXGIAdapter>> enumerateAdapters() {
        std::vector<Microsoft::WRL::ComPtr<IDXGIAdapter>> result;

        UINT i = 0;
        Microsoft::WRL::ComPtr<IDXGIAdapter> adapter;
        while (_dxgiFactory->EnumAdapters(i, &adapter) != DXGI_ERROR_NOT_FOUND)
        {
            result.push_back(adapter);
            adapter = nullptr;
            ++i;
        }
        return result;
    }


    DXGI_ADAPTER_DESC getAdapterDesc(Microsoft::WRL::ComPtr<IDXGIAdapter> adapter) {
        DXGI_ADAPTER_DESC desc;
        if (adapter->GetDesc(&desc) != S_OK) {
            throw std::runtime_error("Failed to get adapter desc");
        }
        return desc;
    }

    LUID getAdapterLuid(Microsoft::WRL::ComPtr<IDXGIAdapter> adapter) {
        DXGI_ADAPTER_DESC desc = getAdapterDesc(adapter);
        return desc.AdapterLuid;
    }
private:
    Microsoft::WRL::ComPtr<IDXGIFactory> _dxgiFactory;
};
