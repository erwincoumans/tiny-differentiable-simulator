"""Optional pyimgui 2.x integration for a single pytinyopengl3 window.

Create after TinyOpenGL3App, call new_frame() before widgets, render() after
render_scene(), then app.swap_buffer(). Call shutdown() before destroying app.
The native renderer currently supports one window (its callbacks are global).
"""
import os
import sys
import time

# Tiny's desktop Linux window uses GLX, including under XWayland. PyOpenGL
# otherwise auto-selects EGL on Wayland, which cannot see the GLX context.
# An explicitly selected platform (e.g. an offscreen EGL app) is respected.
if sys.platform.startswith('linux'):
    os.environ.setdefault('PYOPENGL_PLATFORM', 'glx')

import imgui
from imgui.integrations.opengl import ProgrammablePipelineRenderer
import pytinyopengl3 as tiny


class ImGuiRenderer:
    def __init__(self, app):
        self.app = app  # Keep the native window/context alive until shutdown.
        self.window = app.window
        if not hasattr(self.window, 'get_keyboard_callback'):
            raise RuntimeError('Rebuild pytinyopengl3 for ImGui callback support')
        self.previous_context = imgui.get_current_context()
        self.context = imgui.create_context()
        self.io = imgui.get_io()
        self.io.ini_file_name = None
        self.renderer = ProgrammablePipelineRenderer()
        self.previous = {}
        self.forwarded_buttons = set()
        self.forwarded_keys = set()
        self.mouse_down = [False] * 5
        self.mouse_pressed = [False] * 5
        self.last_time = None
        self.closed = False
        for name in ('TAB', 'LEFT_ARROW', 'RIGHT_ARROW', 'UP_ARROW', 'DOWN_ARROW',
                     'PAGE_UP', 'PAGE_DOWN', 'HOME', 'END', 'INSERT', 'DELETE',
                     'BACKSPACE', 'SPACE', 'ESCAPE'):
            self.io.key_map[getattr(imgui, 'KEY_' + name)] = getattr(tiny, 'TINY_KEY_' + name)
        self.io.key_map[imgui.KEY_ENTER] = tiny.TINY_KEY_RETURN
        for name in ('A', 'C', 'V', 'X', 'Y', 'Z'):
            self.io.key_map[getattr(imgui, 'KEY_' + name)] = ord(name.lower())
        for name in ('keyboard', 'mouse_move', 'mouse_button', 'wheel'):
            self.previous[name] = getattr(self.window, 'get_' + name + '_callback')()
            getattr(self.window, 'set_' + name + '_callback')(getattr(self, '_' + name))

    def _activate(self):
        imgui.set_current_context(self.context)

    def _forward(self, name, *args):
        callback = self.previous[name]
        if callback:
            callback(*args)

    def _mouse_move(self, x, y):
        self._activate()
        self.io.mouse_pos = x, y
        if self.forwarded_buttons or not self.io.want_capture_mouse:
            self._forward('mouse_move', x, y)

    def _mouse_button(self, button, state, x, y):
        self._activate()
        self.io.mouse_pos = x, y
        # Tiny uses left=0, middle=1, right=2; ImGui swaps the latter two.
        if 0 <= button < 5:
            imgui_button = {1: 2, 2: 1}.get(button, button)
            self.mouse_down[imgui_button] = bool(state)
            self.mouse_pressed[imgui_button] |= bool(state)
        if state and not self.io.want_capture_mouse:
            self.forwarded_buttons.add(button)
            self._forward('mouse_button', button, state, x, y)
        elif not state and button in self.forwarded_buttons:
            self.forwarded_buttons.remove(button)
            self._forward('mouse_button', button, state, x, y)

    def _wheel(self, dx, dy):
        self._activate()
        self.io.mouse_wheel_horizontal += dx
        self.io.mouse_wheel += dy
        if not self.io.want_capture_mouse:
            self._forward('wheel', dx, dy)

    def _keyboard(self, key, state):
        self._activate()
        if 0 <= key < len(self.io.keys_down):
            self.io.keys_down[key] = bool(state)
        self._modifiers()
        # Tiny exposes key events, not a Unicode/IME text stream. Support ASCII
        # entry here; applications needing IME should provide a text backend.
        if state and 32 <= key <= 126 and not (self.io.key_ctrl or self.io.key_alt):
            char = chr(key)
            if self.io.key_shift:
                shifted = dict(zip('`1234567890-=[]\\;\',./', '~!@#$%^&*()_+{}|:"<>?'))
                char = shifted.get(char, char.upper())
            self.io.add_input_character(ord(char))
        if state and not self.io.want_capture_keyboard:
            self.forwarded_keys.add(key)
            self._forward('keyboard', key, state)
        elif not state and key in self.forwarded_keys:
            self.forwarded_keys.remove(key)
            self._forward('keyboard', key, state)

    def _modifiers(self):
        for attr, name in (('key_ctrl', 'CONTROL'), ('key_shift', 'SHIFT'), ('key_alt', 'ALT')):
            setattr(self.io, attr, self.window.is_modifier_key_pressed(getattr(tiny, 'TINY_KEY_' + name)))

    def new_frame(self):
        self._activate()
        now = time.perf_counter()
        self.io.delta_time = max(now - self.last_time, 1e-6) if self.last_time else 1 / 60
        self.last_time = now
        self.io.display_size = max(1, self.window.get_width()), max(1, self.window.get_height())
        scale = self.window.get_retina_scale()
        self.io.display_fb_scale = scale, scale
        for i in range(5):
            self.io.mouse_down[i] = self.mouse_down[i] or self.mouse_pressed[i]
            self.mouse_pressed[i] = False
        self._modifiers()
        imgui.new_frame()

    def render(self):
        self._activate()
        imgui.render()
        self.renderer.render(imgui.get_draw_data())

    def shutdown(self):
        if self.closed:
            return
        self._activate()
        # Release camera gestures before restoring callbacks.
        for button in self.forwarded_buttons:
            self._forward('mouse_button', button, 0, *self.io.mouse_pos)
        for key in self.forwarded_keys:
            self._forward('keyboard', key, 0)
        for name, callback in self.previous.items():
            getattr(self.window, 'set_' + name + '_callback')(callback)
        self.renderer.shutdown()
        imgui.destroy_context(self.context)
        if self.previous_context:
            imgui.set_current_context(self.previous_context)
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.shutdown()
