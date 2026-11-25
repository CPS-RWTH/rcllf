// Copyright 2026 rcllf Contributors
// Licensed under the Apache License, Version 2.0
// Stand-in vendor SDK header for the unresolved-include fixture. The test copies it to
// build/blind_pkg/vendor_sdk-1.0/include/vendor_sdk/sdk.h to model an SDK that
// a package build unpacked but did not install.
#ifndef VENDOR_SDK__SDK_H_
#define VENDOR_SDK__SDK_H_
typedef struct sdk_handle sdk_handle_t;
double sdk_step(sdk_handle_t * h);
void sdk_set(sdk_handle_t * h, double v);
#endif  // VENDOR_SDK__SDK_H_
