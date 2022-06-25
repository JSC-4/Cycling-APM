/*
 * Copyright (c) 2021, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*********** WARNING: This is an auto-generated file. Do not edit! ***********/

#ifndef __CONFIG_IMPL_H__
#define __CONFIG_IMPL_H__

/* IPC model */
#define CONFIG_TFM_SPM_BACKEND_IPC                               1

/* Trustzone NS agent working stack size. */
#define CONFIG_TFM_NS_AGENT_TZ_STACK_SIZE                        1024

#if TFM_LVL > 1
#define CONFIG_TFM_PSA_API_SUPERVISOR_CALL                       1
#else /* TFM_LVL > 1 */
#define CONFIG_TFM_PSA_API_THREAD_CALL                           1

/* SPM re-uses Trustzone NS agent stack. */
#define CONFIG_TFM_SPM_THREAD_STACK_SIZE                             \
            CONFIG_TFM_NS_AGENT_TZ_STACK_SIZE                       
#endif /* TFM_LVL > 1 */


#if (CONFIG_TFM_SPE_FP > 0) && (CONFIG_TFM_SPM_BACKEND_SFN == 1)
#error "FP is not supported for SFN model."
#endif

#include "psa_interface_redirect.h"

#endif /* __CONFIG_IMPL_H__ */
