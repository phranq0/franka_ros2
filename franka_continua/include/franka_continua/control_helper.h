// Copyright 2021 wngfra.
// SPDX-License-Identifier: Apache-2.0

#include <exception>
#include <iostream>
#include <sys/utsname.h>

#include <franka/control_types.h>

/**
 * @brief Get the Realtime Config based on kernel version.
 * Only works for linux; if the kernel is not patched, get non-realtime config.
 * 
 * @return franka::RealtimeConfig
 */
franka::RealtimeConfig GetRealtimeConfig()
{
    struct utsname buffer;
    if (uname(&buffer) == 0)
    {
        try
        {
            std::string version(buffer.version);
            if (version.find("PREEMPT_RT") == std::string::npos)
            {
                printf("Setting non-realtime config.\n");
                return franka::RealtimeConfig::kIgnore;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
    printf("Setting realtime config.\n");
    return franka::RealtimeConfig::kEnforce;
}