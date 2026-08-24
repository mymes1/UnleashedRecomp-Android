#pragma once

#include <cstdint>

// First-person "head cam" (issue #128): binds the camera to Sonic's head.
//
// The orientation is deliberately decoupled from the character's rotation:
// the view keeps a stable horizon and only follows the facing direction, so
// a spin-jump reads as a jump (the camera follows the arc) instead of the
// whole world spinning with the character.
namespace HeadCam
{
    // Called after the original SWA::CCamera::UpdateSerial returns.
    // r3 is the CCamera being serialized.
    void OnCameraUpdateSerial(uint32_t camera, uint8_t* base);

    // Called just before the original SWA::CCamera::UpdateSerial runs.
    void OnCameraUpdateSerialPre(uint32_t camera, uint8_t* base);

    // The D3D9 device was created at this guest address.
    void OnDeviceCreated(uint32_t device);

    // The update director (SWA game update) is ticking; r3 is its object.
    void OnUpdateDirector(uint32_t director, uint8_t* base);

    // The D3D GetViewport helper was called; the argument is host memory.
    void OnGetViewport(void* object);

    // Reset transient state (option toggled). Safe to call from any thread;
    // the actual reset happens on the game thread.
    void Reset();
}
