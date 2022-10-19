/*
 * This file is part of AtomGL.
 *
 * Copyright 2022 Davide Bettio <davide@uninstall.it>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <context.h>
#include <interop.h>

#include <esp32_sys.h>

Context *acep_5in65_7c_display_driver_create_port(GlobalContext *global, term opts);
Context *ili934x_display_create_port(GlobalContext *global, term opts);
Context *memory_lcd_display_create_port(GlobalContext *global, term opts);

Context *display_create_port(GlobalContext *global, term opts)
{
    int compat_atom_index = globalcontext_insert_atom(global, ATOM_STR("\x6", "compat"));
    term compat_atom = term_from_atom_index(compat_atom_index);

    term compat_value_term = interop_proplist_get_value(opts, compat_atom);
    if (compat_value_term == term_nil()) {
        return NULL;
    }

    int ok;
    char *compat_string = interop_term_to_string(compat_value_term, &ok);

    if (!ok) {
        return NULL;
    }

    Context *ctx = NULL;
    if (!strcmp(compat_string, "acep_5in65_7c")) {
        ctx = acep_5in65_7c_display_driver_create_port(global, opts);
    } else if (!strcmp(compat_string, "sharp_memory_lcd")) {
        ctx = memory_lcd_display_create_port(global, opts);
    } else if (!strcmp(compat_string, "ili934x")) {
        ctx = ili934x_display_create_port(global, opts);
    }

    free(compat_string);

    return ctx;
}

REGISTER_PORT_DRIVER(display, NULL, display_create_port)
