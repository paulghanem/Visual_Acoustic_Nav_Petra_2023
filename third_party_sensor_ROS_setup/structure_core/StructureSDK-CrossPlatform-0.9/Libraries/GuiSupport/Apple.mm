/*
    Apple.m

    Copyright © 2020 Occipital, Inc. All rights reserved.
    This file is part of the Structure SDK.
    Unauthorized copying of this file, via any medium is strictly prohibited.
    Proprietary and confidential.

    http://structure.io
*/

#if !__APPLE__
#error "This file should only be compiled on macOS."
#endif

#include "Apple.h"
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_NSGL
#include <GLFW/glfw3native.h>
#import <AppKit/AppKit.h>

void GuiSupport::updateNSGLContextForWindow(struct GLFWwindow *window) {
    NSOpenGLContext *ctx = (__bridge id)glfwGetNSGLContext(window);
    [ctx update];
}
