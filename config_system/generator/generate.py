# Copyright (c) 2015 Ambroz Bizjak
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import sys
import os
sys.path.insert(1, os.path.join(os.path.dirname(__file__), '../utils'))
import argparse
import json
import re
import string
import config_reader
import selection
import function_defined
import assign_func
import rich_template
import file_utils
import nix_utils

IDENTIFIER_REGEX = '\\A[A-Za-z][A-Za-z0-9_]{0,127}\\Z'

class GenState(object):
    def __init__ (self):
        self._subst = {}
        self._config_options = []
        self._constants = []
        self._platform_includes = []
        self._aprinter_includes = set()
        self._objects = {}
        self._singleton_objects = {}
        self._finalize_actions = []
        self._global_code = []
        self._init_calls = []
        self._final_init_calls = []
        self._global_resources = []
        self._modules_exprs = []
        self._build_vars = {}
        self._extra_sources = []
        self._extra_include_paths = []
        self._need_millisecond_clock = False
        self._have_hw_millisecond_clock = False
        self._defines = []
        self._linker_symbols = []
    
    def add_subst (self, key, val, indent=-1):
        self._subst[key] = {'val':val, 'indent':indent}
    
    def add_config (self, name, dtype, value, is_constant=False, is_complex=False):
        properties = []
        if is_constant:
            properties.append('ConfigPropertyConstant')
        properties_str = 'ConfigProperties<{}>'.format(', '.join(properties))
        if dtype == 'double':
            config_option_str = 'APRINTER_CONFIG_OPTION_DOUBLE({}, {}, {})'.format(name, value, properties_str)
        elif is_complex:
            config_option_str = 'APRINTER_CONFIG_OPTION_COMPLEX({}, {}, APRINTER_WRAP_COMPLEX_VALUE({}, {}), {})'.format(name, dtype, dtype, value, properties_str)
        else:
            config_option_str = 'APRINTER_CONFIG_OPTION_SIMPLE({}, {}, {}, {})'.format(name, dtype, value, properties_str)
        self._config_options.append(config_option_str)
        return name
    
    def add_float_config (self, name, value, **kwargs):
        return self.add_config(name, 'double', format_cpp_float(value), **kwargs)
    
    def add_bool_config (self, name, value, **kwargs):
        return self.add_config(name, 'bool', 'true' if value else 'false', **kwargs)
    
    def add_mac_addr_config (self, name, value, **kwargs):
        assert len(value) == 6
        val_str = '(ConfigTypeMacAddress{{{{{}}}}})'.format(', '.join('0x{:02x}'.format(x) for x in value))
        return self.add_config(name, 'ConfigTypeMacAddress', val_str, is_complex=True, **kwargs)
    
    def add_ip_addr_config (self, name, value, **kwargs):
        assert len(value) == 4
        val_str = '(ConfigTypeIpAddress{{{{{}}}}})'.format(', '.join('{}'.format(x) for x in value))
        return self.add_config(name, 'ConfigTypeIpAddress', val_str, is_complex=True, **kwargs)
    
    def add_float_constant (self, name, value):
        self._constants.append({'type':'using', 'name':name, 'value':'AMBRO_WRAP_DOUBLE({})'.format(format_cpp_float(value))})
        return name
    
    def add_typedef (self, name, value):
        self._constants.append({'type':'using', 'name':name, 'value':value})
        return name
    
    def add_int_constant (self, dtype, name, value):
        if dtype == 'int':
            c_type = 'int'
            c_init = str(value)
        else:
            m = re.match('\\A(u?)int(8|16|32|64)\\Z', dtype)
            assert m
            u = m.group(1)
            b = m.group(2)
            c_type = '{}_t'.format(dtype)
            c_init = '{}INT{}_C({})'.format(u.upper(), b, value)
        
        self._constants.append({
            'type': 'static {} const'.format(c_type),
            'name': name,
            'value': c_init,
        })
        return name
    
    def add_platform_include (self, inc_file):
        self._platform_includes.append(inc_file)
    
    def add_include (self, inc_file):
        self._aprinter_includes.add(inc_file)
    
    def add_aprinter_include (self, inc_file):
        self.add_include('aprinter/'+inc_file)
    
    def register_objects (self, kind, config, key):
        if kind not in self._objects:
            self._objects[kind] = {}
        for obj_config in config.iter_list_config(key, max_count=20):
            name = obj_config.get_string('Name')
            if name in self._objects[kind]:
                obj_config.path().error('Duplicate {} name'.format(kind))
            self._objects[kind][name] = obj_config
    
    def get_object (self, kind, config, key):
        name = config.get_string(key)
        if kind not in self._objects or name not in self._objects[kind]:
            config.key_path(key).error('Nonexistent {} specified'.format(kind))
        return self._objects[kind][name]
    
    def register_singleton_object (self, kind, value):
        assert kind not in self._singleton_objects
        self._singleton_objects[kind] = value
        return value
    
    def get_singleton_object (self, kind, allow_none=False):
        have = kind in self._singleton_objects
        assert allow_none or have
        return self._singleton_objects[kind] if have else None
    
    def add_global_code (self, priority, code):
        self._global_code.append({'priority':priority, 'code':code})
    
    def add_isr (self, isr):
        self.add_global_code(-1, isr)
    
    def add_init_call (self, priority, init_call):
        self._init_calls.append({'priority':priority, 'init_call':init_call})
    
    def add_final_init_call (self, priority, init_call):
        self._final_init_calls.append({'priority':priority, 'init_call':init_call})
    
    def add_finalize_action (self, action):
        self._finalize_actions.append(action)
    
    def add_global_resource (self, priority, name, expr, context_name=None, code_before=None, code_before_program=None,
                             extra_program_child=None, is_fast_event_root=False, use_instance=False):
        code = ''
        if code_before is not None:
            code += '{}\n'.format(code_before)
        if use_instance:
            code += 'APRINTER_MAKE_INSTANCE({}, ({}))\n'.format(name, expr.build(indent=0))
        else:
            code += 'using {} = {};\n'.format(name, expr.build(indent=0))
        self._global_resources.append({
            'priority': priority,
            'name': name,
            'context_name':context_name,
            'code': code,
            'code_before_program': code_before_program,
            'extra_program_child': extra_program_child,
            'is_fast_event_root': is_fast_event_root,
        })
    
    def add_module (self):
        index = len(self._modules_exprs)
        self._modules_exprs.append(None)
        return GenPrinterModule(self, index)
    
    def add_build_var (self, name, value):
        assert name not in self._build_vars
        self._build_vars[name] = value
    
    def add_extra_source (self, path):
        self._extra_sources.append(path)
    
    def add_extra_include_path (self, path):
        self._extra_include_paths.append(path)
    
    def set_need_millisecond_clock (self):
        self._need_millisecond_clock = True
    
    def set_have_hw_millisecond_clock (self):
        self._have_hw_millisecond_clock = True
    
    def add_define (self, name, value=''):
        self._defines.append({'name': name, 'value': str(value)})
    
    def add_linker_symbol (self, name, value):
        self._linker_symbols.append({'name': name, 'value': str(value)})
    
    def finalize (self):
        for action in reversed(self._finalize_actions):
            action()
        
        for so in self._singleton_objects.itervalues():
            if hasattr(so, 'finalize'):
                so.finalize()
        
        global_resources = sorted(self._global_resources, key=lambda x: x['priority'])
        
        program_children = []
        program_children.extend(gr['name'] for gr in global_resources)
        program_children.extend(gr['extra_program_child'] for gr in global_resources if gr['extra_program_child'] is not None)
        
        self.add_subst('GENERATED_WARNING', 'WARNING: This file was automatically generated!')
        self.add_subst('EXTRA_CONSTANTS', ''.join('{} {} = {};\n'.format(c['type'], c['name'], c['value']) for c in self._constants))
        self.add_subst('ConfigOptions', ''.join('{}\n'.format(c) for c in self._config_options))
        self.add_subst('PLATFORM_INCLUDES', ''.join('#include <{}>\n'.format(inc) for inc in self._platform_includes))
        self.add_subst('AprinterIncludes', ''.join('#include <{}>\n'.format(inc) for inc in sorted(self._aprinter_includes)))
        self.add_subst('GlobalCode', ''.join('{}\n'.format(gc['code']) for gc in sorted(self._global_code, key=lambda x: x['priority'])))
        self.add_subst('InitCalls', ''.join('    {}\n'.format(ic['init_call']) for ic in sorted(self._init_calls, key=lambda x: x['priority'])))
        self.add_subst('GlobalResourceExprs', ''.join(gr['code'] for gr in global_resources))
        self.add_subst('GlobalResourceContextAliases', ''.join('    using {} = ::{};\n'.format(gr['context_name'], gr['name']) for gr in global_resources if gr['context_name'] is not None))
        self.add_subst('GlobalResourceProgramChildren', ',\n'.join('    {}'.format(pc_name) for pc_name in program_children))
        self.add_subst('GlobalResourceInit', ''.join('    {}::init(c);\n'.format(gr['name']) for gr in global_resources))
        self.add_subst('FinalInitCalls', ''.join('    {}\n'.format(ic['init_call']) for ic in sorted(self._final_init_calls, key=lambda x: x['priority'])))
        self.add_subst('CodeBeforeProgram', ''.join('{}\n'.format(gr['code_before_program']) for gr in global_resources if gr['code_before_program'] is not None))
    
    def get_subst (self):
        res = {}
        for (key, subst) in self._subst.iteritems():
            val = subst['val']
            indent = subst['indent']
            res[key] = val if type(val) is str else val.build(indent)
        return res
    
class GenPrinterModule(object):
    def __init__ (self, gen, index):
        self._gen = gen
        self._index = index
    
    @property
    def index (self):
        return self._index
    
    def set_expr (self, expr):
        self._gen._modules_exprs[self._index] = expr

class GenConfigReader(config_reader.ConfigReader):
    def get_int_constant (self, key):
        return str(self.get_int(key))
    
    def get_bool_constant (self, key):
        return 'true' if self.get_bool(key) else 'false'

    def get_float_constant (self, key):
        return format_cpp_float(self.get_float(key))

    def get_identifier (self, key, validate=None):
        val = self.get_string(key)
        if not re.match(IDENTIFIER_REGEX, val):
            self.key_path(key).error('Incorrect format.')
        if validate is not None and not validate(val):
            self.key_path(key).error('Custom validation failed.')
        return val
    
    def get_id_char (self, key):
        val = self.get_string(key)
        if val not in string.ascii_uppercase:
            self.key_path(key).error('Incorrect format.')
        return val
    
    def get_mac_addr (self, key):
        val = self.get_string(key)
        br = '([0-9A-Fa-f]{1,2})'
        mac_re = '\\A{}:{}:{}:{}:{}:{}\\Z'.format(br, br, br, br, br, br)
        m = re.match(mac_re, val)
        if not m:
            self.key_path(key).error('Incorrect format.')
        return [int(m.group(i), 16) for i in range(1, 7)]
    
    def get_ip_addr (self, key):
        val = self.get_string(key)
        br = '([0-9]{1,3})'
        ip_re = '\\A{}\\.{}\\.{}\\.{}\\Z'.format(br, br, br, br)
        m = re.match(ip_re, val)
        if not m:
            self.key_path(key).error('Incorrect format A.')
        ints = [int(m.group(i), 10) for i in range(1, 5)]
        if any(d > 255 for d in ints):
            self.key_path(key).error('Incorrect format B.')
        return ints
    
    def do_selection (self, key, sel_def):
        for config in self.enter_config(key):
            try:
                result = sel_def.run(config.get_string('_compoundName'), config)
            except selection.SelectionError:
                config.path().error('Unknown choice.')
            return result
    
    def do_list (self, key, elem_cb, min_count=-1, max_count=-1):
        elems = []
        for (i, config) in enumerate(self.iter_list_config(key, min_count=min_count, max_count=max_count)):
            elems.append(elem_cb(config, i))
        return TemplateList(elems)
    
    def do_keyed_list (self, count, elems_key, elem_key_prefix, elem_cb):
        elems = []
        elems_config = self.get_config(elems_key)
        for i in range(count):
            elem_config = elems_config.get_config('{}{}'.format(elem_key_prefix, i))
            elems.append(elem_cb(elem_config, i))
        return TemplateList(elems)
    
    def do_enum (self, key, mapping):
        val = self.get_string(key)
        if val not in mapping:
            self.key_path(key).error('Incorrect choice.')
        return mapping[val]

class TemplateExpr(object):
    def __init__ (self, name, args):
        self._name = name
        self._args = args
    
    def append_arg(self, arg):
        self._args.append(arg)
    
    def build (self, indent):
        if indent == -1 or len(self._args) == 0:
            initiator = ''
            separator = ', '
            terminator = ''
            child_indent = -1
        else:
            initiator = '\n' + '    ' * (indent + 1)
            separator = ',' + initiator
            terminator = '\n' + '    ' * indent
            child_indent = indent + 1
        return '{}<{}{}{}>'.format(self._name, initiator, separator.join(_build_template_arg(arg, child_indent) for arg in self._args), terminator)

def _build_template_arg (arg, indent):
    if type(arg) is str or type(arg) is int or type(arg) is long:
        return str(arg)
    if type(arg) is bool:
        return 'true' if arg else 'false'
    return arg.build(indent)

class TemplateList(TemplateExpr):
    def __init__ (self, args):
        TemplateExpr.__init__(self, 'MakeTypeList', args)

class TemplateChar(object):
    def __init__ (self, ch):
        self._ch = ch
    
    def build (self, indent):
        return '\'{}\''.format(self._ch)

class TemplateLiteral(object):
    def __init__ (self, str):
        self._str = str
    
    def build (self, indent):
        return self._str

def format_cpp_float(value):
    return '{:.17E}'.format(value).replace('INF', 'INFINITY')

def setup_event_loop(gen):
    impl_obj = gen.get_singleton_object('event_loop_impl', allow_none=True)
    if impl_obj is None:
        impl = 'BusyEventLoop'
        impl_extra_args = []
    else:
        impl = impl_obj['name']
        impl_extra_args = impl_obj['extra_args']
    
    gen.add_aprinter_include('system/{}.h'.format(impl))
    
    code_before_expr = 'struct MyLoopExtraDelay;\n'
    expr = TemplateExpr('{}Arg'.format(impl), ['Context', 'Program', 'MyLoopExtraDelay'] + impl_extra_args)
    
    fast_events = 'ObjCollect<MakeTypeList<{}>, MemberType_EventLoopFastEvents>'.format(', '.join(gr['name'] for gr in gen._global_resources if gr['is_fast_event_root']))
    
    code_before_program  = 'APRINTER_DEFINE_MEMBER_TYPE(MemberType_EventLoopFastEvents, EventLoopFastEvents)\n'
    code_before_program += 'APRINTER_MAKE_INSTANCE(MyLoopExtra, ({}ExtraArg<Program, MyLoop, {}>))\n'.format(impl, fast_events)
    code_before_program += 'struct MyLoopExtraDelay : public WrapType<MyLoopExtra> {};'
    
    gen.add_global_resource(0, 'MyLoop', expr, use_instance=True, context_name='EventLoop', code_before=code_before_expr, code_before_program=code_before_program, extra_program_child='MyLoopExtra')
    gen.add_final_init_call(100, 'MyLoop::run(c);')

def setup_platform(gen, config, key):
    platform_sel = selection.Selection()
    
    arm_checksum_src_file = 'aprinter/net/inet_chksum_arm.S'
    
    @platform_sel.options(['At91Sam3x8e', 'At91Sam3u4e'])
    def option(platform_name, platform):
        stack_size = platform.get_int('StackSize')
        if not 512 <= stack_size <= 32768:
            platform.key_path('StackSize').error('Value out of range.')
        
        gen.add_platform_include('aprinter/platform/at91sam/at91sam_support.h')
        gen.add_init_call(-1, 'platform_init();')
        gen.register_singleton_object('checksum_src_file', arm_checksum_src_file)
        gen.add_linker_symbol('__stack_size__', stack_size)
    
    @platform_sel.option('Teensy3')
    def option(platform):
        gen.add_platform_include('aprinter/platform/teensy3/teensy3_support.h')
        gen.register_singleton_object('checksum_src_file', arm_checksum_src_file)
    
    @platform_sel.options(['AVR ATmega2560', 'AVR ATmega1284p'])
    def option(platform_name, platform):
        gen.add_platform_include('avr/io.h')
        gen.add_platform_include('aprinter/platform/avr/avr_support.h')
        gen.add_init_call(-3, 'sei();')
    
    @platform_sel.option('Stm32')
    def option(platform):
        gen.add_platform_include('aprinter/platform/stm32/stm32_support.h')
        gen.add_init_call(-1, 'platform_init();')
        gen.add_final_init_call(-1, 'platform_init_final();')
        gen.register_singleton_object('checksum_src_file', arm_checksum_src_file)
    
    @platform_sel.option('Linux')
    def option(platform):
        timers_structure = get_heap_structure(gen, platform, 'TimersStructure')
        
        gen.add_platform_include('aprinter/platform/linux/linux_support.h')
        gen.add_init_call(-1, 'platform_init(argc, argv);')
        gen.register_singleton_object('event_loop_impl', {
            'name': 'LinuxEventLoop',
            'extra_args': [timers_structure],
        })
    
    config.do_selection(key, platform_sel)

def setup_debug_interface(gen, config, key):
    debug_sel = selection.Selection()
    
    @debug_sel.option('NoDebug')
    def option(debug):
        pass
    
    @debug_sel.option('ArmItmDebug')
    def option(debug):
        stimulus_port = debug.get_int('StimulusPort')
        if not 0 <= stimulus_port <= 31:
            debug.key_path('StimulusPort').error('Incorrect value.')
        gen.add_platform_include('aprinter/hal/generic/ArmItmDebug.h')
        gen.add_aprinter_include('hal/generic/NewlibDebugWrite.h')
        gen.add_global_code(0, 'using MyDebug = ArmItmDebug<Context, {}>;'.format(
            stimulus_port,
        ))
        gen.add_global_code(0, 'APRINTER_SETUP_NEWLIB_DEBUG_WRITE(MyDebug::write, Context())')
    
    config.do_selection(key, debug_sel)

class CommonClock(object):
    def __init__ (self, gen, config, clock_name, priority, clockdef_func):
        self._gen = gen
        self._config = config
        self._clock_name = clock_name
        self._my_clock = 'My{}'.format(clock_name)
        self._priority = priority
        self._clockdef = function_defined.FunctionDefinedClass(clockdef_func)
        gen.add_aprinter_include(self._clockdef.INCLUDE)
        self._timers = self._load_timers(config)
        self._interrupt_timers = []
        self._primary_timer = self.check_timer(config.get_string('primary_timer'), config.key_path('primary_timer'))
    
    def _load_timers (self, config):
        timers = {}
        if hasattr(self._clockdef, 'handle_timer'):
            for timer_config in config.iter_list_config('timers', max_count=20):
                timer_id = self.check_timer(timer_config.get_string('Timer'), timer_config.key_path('Timer'))
                if timer_id in timers:
                    timer_config.path().error('Duplicate timer specified.')
                timers[timer_id] = self._clockdef.handle_timer(self._gen, timer_id, timer_config)
        return timers
    
    def check_timer (self, timer_name, path):
        match = re.match(self._clockdef.TIMER_RE, timer_name) 
        if not match:
            path.error('Incorrect timer name.')
        return match.group(1)
    
    def check_oc_unit (self, name, path):
        m = re.match(self._clockdef.CHANNEL_RE, name)
        if m is None:
            path.error('Incorrect OC unit format.')
        return {'tc':m.group(1), 'channel':m.group(2)}
    
    def add_interrupt_timer (self, name, user, clearance, path):
        it = self.check_oc_unit(name, path)
        self._interrupt_timers.append(it)
        clearance_name = '{}_{}_Clearance'.format(self._my_clock, name)
        self._gen.add_float_constant(clearance_name, clearance)
        self._gen.add_isr(self._clockdef.INTERRUPT_TIMER_ISR(it, user))
        return self._clockdef.INTERRUPT_TIMER_EXPR(it, clearance_name)
    
    def finalize (self):
        auto_timers = (set(it['tc'] for it in self._interrupt_timers) | set([self._primary_timer])) - set(self._timers)
        for timer_id in auto_timers:
            self._timers[timer_id] = self._clockdef.TIMER_EXPR(timer_id)
            if hasattr(self._clockdef, 'TIMER_ISR'):
                self._gen.add_isr(self._clockdef.TIMER_ISR(self._my_clock, timer_id))
        
        if hasattr(self._clockdef, 'CLOCK_ISR'):
            clock = {'primary_timer': self._primary_timer}
            self._gen.add_isr(self._clockdef.CLOCK_ISR(self._my_clock, clock))
        
        temp_timers = set(self._timers)
        temp_timers.remove(self._primary_timer)
        ordered_timers = [self._primary_timer] + sorted(temp_timers)
        timers_expr = TemplateList([self._timers[timer_id] for timer_id in ordered_timers])
        
        clock_service_expr = self._clockdef.CLOCK_SERVICE(self._config)
        service_code = 'using {}Service = {};'.format(self._my_clock, clock_service_expr.build(indent=0))
        clock_expr = TemplateExpr('{}Service::Clock'.format(self._my_clock), ['Context', 'Program', timers_expr])
        self._gen.add_global_resource(self._priority, self._my_clock, clock_expr, use_instance=True, code_before=service_code, context_name=self._clock_name)

def At91Sam3xClockDef(x):
    x.INCLUDE = 'hal/at91/At91Sam3xClock.h'
    x.CLOCK_SERVICE = lambda config: TemplateExpr('At91Sam3xClockService', [config.get_int_constant('prescaler')])
    x.TIMER_RE = '\\ATC([0-9])\\Z'
    x.CHANNEL_RE = '\\ATC([0-9])([A-C])\\Z'
    x.INTERRUPT_TIMER_EXPR = lambda it, clearance: 'At91Sam3xClockInterruptTimerService<At91Sam3xClockTC{}, At91Sam3xClockComp{}, {}>'.format(it['tc'], it['channel'], clearance)
    x.INTERRUPT_TIMER_ISR = lambda it, user: 'AMBRO_AT91SAM3X_CLOCK_INTERRUPT_TIMER_GLOBAL(At91Sam3xClockTC{}, At91Sam3xClockComp{}, {}, Context())'.format(it['tc'], it['channel'], user)
    x.TIMER_EXPR = lambda tc: 'At91Sam3xClockTC{}'.format(tc)
    x.TIMER_ISR = lambda my_clock, tc: 'AMBRO_AT91SAM3X_CLOCK_TC{}_GLOBAL({}, Context())'.format(tc, my_clock)

def At91Sam3uClockDef(x):
    x.INCLUDE = 'hal/at91/At91Sam3uClock.h'
    x.CLOCK_SERVICE = lambda config: TemplateExpr('At91Sam3uClockService', [config.get_int_constant('prescaler')])
    x.TIMER_RE = '\\ATC([0-9])\\Z'
    x.CHANNEL_RE = '\\ATC([0-9])([A-C])\\Z'
    x.INTERRUPT_TIMER_EXPR = lambda it, clearance: 'At91Sam3uClockInterruptTimerService<At91Sam3uClockTC{}, At91Sam3uClockComp{}, {}>'.format(it['tc'], it['channel'], clearance)
    x.INTERRUPT_TIMER_ISR = lambda it, user: 'AMBRO_AT91SAM3U_CLOCK_INTERRUPT_TIMER_GLOBAL(At91Sam3uClockTC{}, At91Sam3uClockComp{}, {}, Context())'.format(it['tc'], it['channel'], user)
    x.TIMER_EXPR = lambda tc: 'At91Sam3uClockTC{}'.format(tc)
    x.TIMER_ISR = lambda my_clock, tc: 'AMBRO_AT91SAM3U_CLOCK_TC{}_GLOBAL({}, Context())'.format(tc, my_clock)

def Mk20ClockDef(x):
    x.INCLUDE = 'hal/teensy3/Mk20Clock.h'
    x.CLOCK_SERVICE = lambda config: TemplateExpr('Mk20ClockService', [config.get_int_constant('prescaler')])
    x.TIMER_RE = '\\AFTM([0-9])\\Z'
    x.CHANNEL_RE = '\\AFTM([0-9])_([0-9])\\Z'
    x.INTERRUPT_TIMER_EXPR = lambda it, clearance: 'Mk20ClockInterruptTimerService<Mk20ClockFTM{}, {}, {}>'.format(it['tc'], it['channel'], clearance)
    x.INTERRUPT_TIMER_ISR = lambda it, user: 'AMBRO_MK20_CLOCK_INTERRUPT_TIMER_GLOBAL(Mk20ClockFTM{}, {}, {}, Context())'.format(it['tc'], it['channel'], user)
    x.TIMER_EXPR = lambda tc: 'Mk20ClockFtmSpec<Mk20ClockFTM{}>'.format(tc)
    x.TIMER_ISR = lambda my_clock, tc: 'AMBRO_MK20_CLOCK_FTM_GLOBAL({}, {}, Context())'.format(tc, my_clock)

def AvrClockDef(x):
    x.INCLUDE = 'hal/avr/AvrClock.h'
    x.CLOCK_SERVICE = lambda config: TemplateExpr('AvrClockService', [config.get_int_constant('PrescaleDivide')])
    x.TIMER_RE = '\\ATC([0-9])\\Z'
    x.CHANNEL_RE = '\\ATC([0-9])_([A-Z])\\Z'
    x.INTERRUPT_TIMER_EXPR = lambda it, clearance: 'AvrClockInterruptTimerService<AvrClockTcChannel{}{}, {}>'.format(it['tc'], it['channel'], clearance)
    x.INTERRUPT_TIMER_ISR = lambda it, user: 'AMBRO_AVR_CLOCK_INTERRUPT_TIMER_ISRS({}, {}, {}, Context())'.format(it['tc'], it['channel'], user)
    x.TIMER_EXPR = lambda tc: 'AvrClockTcSpec<AvrClockTc{}>'.format(tc)
    x.CLOCK_ISR = lambda my_clock, clock: 'AMBRO_AVR_CLOCK_ISRS({}, {}, Context())'.format(clock['primary_timer'], my_clock)
    
    @assign_func.assign_func(x, 'handle_timer')
    def handle_timer(gen, timer_id, timer_config):
        mode_sel = selection.Selection()
        
        @mode_sel.option('AvrClockTcModeClock')
        def option(mode):
            return 'AvrClockTcModeClock'
        
        @mode_sel.option('AvrClockTcMode8BitPwm')
        def option(mode):
            return TemplateExpr('AvrClockTcMode8BitPwm', [
                mode.get_int('PrescaleDivide'),
            ])
        
        @mode_sel.option('AvrClockTcMode16BitPwm')
        def option(mode):
            return TemplateExpr('AvrClockTcMode16BitPwm', [
                mode.get_int('PrescaleDivide'),
                mode.get_int('TopVal'),
            ])
        
        return TemplateExpr('AvrClockTcSpec', [
            'AvrClockTc{}'.format(timer_id),
            timer_config.do_selection('Mode', mode_sel),
        ])

def Stm32ClockDef(x):
    x.INCLUDE = 'hal/stm32/Stm32Clock.h'
    x.CLOCK_SERVICE = lambda config: TemplateExpr('Stm32ClockService', [config.get_int_constant('prescaler')])
    x.TIMER_RE = '\\ATIM([0-9]{1,2})\\Z'
    x.CHANNEL_RE = '\\ATIM([0-9]{1,2})_([1-4])\\Z'
    x.INTERRUPT_TIMER_EXPR = lambda it, clearance: 'Stm32ClockInterruptTimerService<Stm32ClockTIM{}, Stm32ClockComp{}, {}>'.format(it['tc'], it['channel'], clearance)
    x.INTERRUPT_TIMER_ISR = lambda it, user: 'AMBRO_STM32_CLOCK_INTERRUPT_TIMER_GLOBAL(Stm32ClockTIM{}, Stm32ClockComp{}, {}, Context())'.format(it['tc'], it['channel'], user)
    x.TIMER_EXPR = lambda tc: 'Stm32ClockTIM{}'.format(tc)
    x.TIMER_ISR = lambda my_clock, tc: 'AMBRO_STM32_CLOCK_TC_GLOBAL({}, {}, Context())'.format(tc, my_clock)

def LinuxClockDef(x):
    x.INCLUDE = 'hal/linux/LinuxClock.h'
    x.CLOCK_SERVICE = lambda config: TemplateExpr('LinuxClockService', [config.get_int_constant('SubSecondBits'), config.get_int_constant('MaxTimers')])
    x.TIMER_RE = '\\A()\\Z'
    x.CHANNEL_RE = '\\A()([0-9]{1,2})\\Z'
    x.INTERRUPT_TIMER_EXPR = lambda it, clearance: 'LinuxClockInterruptTimerService<{}, {}>'.format(it['channel'], clearance)
    x.INTERRUPT_TIMER_ISR = lambda it, user: ''
    x.TIMER_EXPR = lambda tc: 'void'

def setup_clock(gen, config, key, clock_name, priority, allow_disabled):
    clock_sel = selection.Selection()
    
    @clock_sel.option('NoClock')
    def option(clock):
        if not allow_disabled:
            clock.path().error('A clock is required.')
        return None
    
    @clock_sel.option('At91Sam3xClock')
    def option(clock):
        return CommonClock(gen, clock, clock_name, priority, At91Sam3xClockDef)
    
    @clock_sel.option('At91Sam3uClock')
    def option(clock):
        return CommonClock(gen, clock, clock_name, priority, At91Sam3uClockDef)
    
    @clock_sel.option('Mk20Clock')
    def option(clock):
        return CommonClock(gen, clock, clock_name, priority, Mk20ClockDef)
    
    @clock_sel.option('AvrClock')
    def option(clock):
        return CommonClock(gen, clock, clock_name, priority, AvrClockDef)
    
    @clock_sel.option('Stm32Clock')
    def option(clock):
        return CommonClock(gen, clock, clock_name, priority, Stm32ClockDef)
    
    @clock_sel.option('LinuxClock')
    def option(clock):
        return CommonClock(gen, clock, clock_name, priority, LinuxClockDef)
    
    clock_object = config.do_selection(key, clock_sel)
    if clock_object is not None:
        gen.register_singleton_object(clock_name, clock_object)

def setup_millisecond_clock(gen, config, key, priority):
    clock_sel = selection.Selection()
    
    @clock_sel.option('NoMillisecondClock')
    def option(clock):
        return None
    
    @clock_sel.option('ArmSysTickMillisecondClock')
    def option(clock):
        gen.add_aprinter_include('hal/generic/ArmSysTickMillisecondClock.h')
        gen.add_isr('APRINTER_ARM_SYSTICK_MILLISECOND_CLOCK_GLOBAL(MyMillisecondClock, Context())')
        gen.set_have_hw_millisecond_clock()
        return TemplateExpr('ArmSysTickMillisecondClock', ['Context', 'Program'])
    
    clock_expr = config.do_selection(key, clock_sel)
    if clock_expr is not None:
        gen.add_global_resource(priority, 'MyMillisecondClock', clock_expr, context_name='MillisecondClock')

def setup_pins (gen, config, key):
    pin_regexes = [IDENTIFIER_REGEX]
    
    pins_sel = selection.Selection()
    
    @pins_sel.option('At91SamPins')
    def options(pin_config):
        gen.add_aprinter_include('hal/at91/At91SamPins.h')
        pin_regexes.append('\\AAt91SamPin<At91SamPio[A-Z],[0-9]{1,3}>\\Z')
        return TemplateLiteral('At91SamPinsService')
    
    @pins_sel.option('Mk20Pins')
    def options(pin_config):
        gen.add_aprinter_include('hal/teensy3/Mk20Pins.h')
        pin_regexes.append('\\AMk20Pin<Mk20Port[A-Z],[0-9]{1,3}>\\Z')
        return TemplateLiteral('Mk20PinsService')
    
    @pins_sel.option('AvrPins')
    def options(pin_config):
        gen.add_aprinter_include('hal/avr/AvrPins.h')
        pin_regexes.append('\\AAvrPin<AvrPort[A-Z],[0-9]{1,3}>\\Z')
        return TemplateLiteral('AvrPinsService')
    
    @pins_sel.option('Stm32Pins')
    def options(pin_config):
        gen.add_aprinter_include('hal/stm32/Stm32Pins.h')
        pin_regexes.append('\\AStm32Pin<Stm32Port[A-Z],[0-9]{1,3}>\\Z')
        return TemplateLiteral('Stm32PinsService')
    
    @pins_sel.option('StubPins')
    def options(pin_config):
        gen.add_aprinter_include('hal/generic/StubPins.h')
        pin_regexes.append('\\AStubPin\\Z')
        return TemplateLiteral('StubPinsService')
    
    service_expr = config.do_selection(key, pins_sel)
    service_code = 'using PinsService = {};'.format(service_expr.build(indent=0))
    pins_expr = TemplateExpr('PinsService::Pins', ['Context', 'Program'])
    gen.add_global_resource(10, 'Pins', pins_expr, use_instance=True, code_before=service_code, context_name='Pins')
    gen.register_singleton_object('pin_regexes', pin_regexes)

def get_pin (gen, config, key):
    pin = config.get_string(key)
    pin_regexes = gen.get_singleton_object('pin_regexes')
    if not any(re.match(pin_regex, pin) for pin_regex in pin_regexes):
        config.key_path(key).error('Invalid pin value.')
    return pin

def setup_watchdog (gen, config, key, user):
    watchdog_sel = selection.Selection()
    
    @watchdog_sel.option('At91SamWatchdog')
    def option(watchdog):
        gen.add_aprinter_include('hal/at91/At91SamWatchdog.h')
        return TemplateExpr('At91SamWatchdogService', [
            watchdog.get_int('Wdv')
        ])
    
    @watchdog_sel.option('Mk20Watchdog')
    def option(watchdog):
        gen.add_aprinter_include('hal/teensy3/Mk20Watchdog.h')
        gen.add_isr('AMBRO_MK20_WATCHDOG_GLOBAL({})'.format(user))
        return TemplateExpr('Mk20WatchdogService', [
            watchdog.get_int('Toval'),
            watchdog.get_int('Prescval'),
        ])
    
    @watchdog_sel.option('AvrWatchdog')
    def option(watchdog):
        wdto = watchdog.get_string('Timeout')
        if not re.match('\\AWDTO_[0-9A-Z]{1,10}\\Z', wdto):
            watchdog.key_path('Timeout').error('Incorrect value.')
        
        gen.add_aprinter_include('hal/avr/AvrWatchdog.h')
        gen.add_isr('AMBRO_AVR_WATCHDOG_GLOBAL')
        return TemplateExpr('AvrWatchdogService', [wdto])
    
    @watchdog_sel.option('Stm32Watchdog')
    def option(watchdog):
        gen.add_aprinter_include('hal/stm32/Stm32Watchdog.h')
        return TemplateExpr('Stm32WatchdogService', [
            watchdog.get_int('Divider'),
            watchdog.get_int('Reload'),
        ])
    
    @watchdog_sel.option('NullWatchdog')
    def option(watchdog):
        gen.add_aprinter_include('hal/generic/NullWatchdog.h')
        return 'NullWatchdogService'
    
    return config.do_selection(key, watchdog_sel)

def setup_adc (gen, config, key):
    adc_sel = selection.Selection()
    
    @adc_sel.option('NoAdc')
    def option(adc_config):
        return None
    
    @adc_sel.option('At91SamAdc')
    def option(adc_config):
        gen.add_aprinter_include('hal/at91/At91SamAdc.h')
        gen.add_float_constant('AdcFreq', adc_config.get_float('freq'))
        gen.add_float_constant('AdcAvgInterval', adc_config.get_float('avg_interval'))
        gen.add_int_constant('uint16', 'AdcSmoothing', max(0, min(65535, int(adc_config.get_float('smoothing') * 65536.0))))
        gen.add_isr('AMBRO_AT91SAM_ADC_GLOBAL(MyAdc, Context())')
        
        return {
            'service_expr': TemplateExpr('At91SamAdcService', [
                'AdcFreq',
                adc_config.get_int('startup'),
                adc_config.get_int('settling'),
                adc_config.get_int('tracking'),
                adc_config.get_int('transfer'),
                'At91SamAdcAvgParams<AdcAvgInterval>',
            ]),
            'pin_func': lambda pin: 'At91SamAdcSmoothPin<{}, AdcSmoothing>'.format(pin)
        }
    
    @adc_sel.option('At91Sam3uAdc')
    def option(adc_config):
        gen.add_aprinter_include('hal/at91/At91SamAdc.h')
        gen.add_float_constant('AdcFreq', adc_config.get_float('freq'))
        gen.add_float_constant('AdcAvgInterval', adc_config.get_float('avg_interval'))
        gen.add_int_constant('uint16', 'AdcSmoothing', max(0, min(65535, int(adc_config.get_float('smoothing') * 65536.0))))
        gen.add_isr('AMBRO_AT91SAM3U_ADC_GLOBAL(MyAdc, Context())')
        
        return {
            'service_expr': TemplateExpr('At91Sam3uAdcService', [
                'AdcFreq',
                adc_config.get_int('startup'),
                adc_config.get_int('shtim'),
                'At91SamAdcAvgParams<AdcAvgInterval>',
            ]),
            'pin_func': lambda pin: 'At91SamAdcSmoothPin<{}, AdcSmoothing>'.format(pin)
        }
    
    @adc_sel.option('Mk20Adc')
    def option(adc_config):
        gen.add_aprinter_include('hal/teensy3/Mk20Adc.h')
        gen.add_int_constant('int32', 'AdcADiv', adc_config.get_int('AdcADiv'))
        gen.add_isr('AMBRO_MK20_ADC_ISRS(MyAdc, Context())')
        
        return {
            'service_expr': TemplateExpr('Mk20AdcService', ['AdcADiv']),
            'pin_func': lambda pin: pin
        }
    
    @adc_sel.option('AvrAdc')
    def option(adc_config):
        gen.add_aprinter_include('hal/avr/AvrAdc.h')
        gen.add_int_constant('int32', 'AdcRefSel', adc_config.get_int('RefSel'))
        gen.add_int_constant('int32', 'AdcPrescaler', adc_config.get_int('Prescaler'))
        gen.add_int_constant('int32', 'AdcOverSamplingBits', adc_config.get_int('OverSamplingBits'))
        gen.add_isr('AMBRO_AVR_ADC_ISRS(MyAdc, Context())')
        
        return {
            'service_expr': TemplateExpr('AvrAdcService', ['AdcRefSel', 'AdcPrescaler', 'AdcOverSamplingBits']),
            'pin_func': lambda pin: pin
        }
    
    @adc_sel.option('Stm32Adc')
    def option(adc_config):
        gen.add_aprinter_include('hal/stm32/Stm32Adc.h')
        gen.add_isr('APRINTER_STM32_ADC_GLOBAL(MyAdc, Context())')
        
        return {
            'service_expr': TemplateExpr('Stm32AdcService', [
                adc_config.get_int('ClockDivider'),
                adc_config.get_int('SampleTimeSelection'),
            ]),
            'pin_func': lambda pin: pin
        }
    
    result = config.do_selection(key, adc_sel)
    if result is None:
        return
    
    gen.register_singleton_object('adc_pins', [])
    
    def finalize():
        adc_pins = gen.get_singleton_object('adc_pins')
        pins_expr = TemplateList([result['pin_func'](pin) for pin in adc_pins])
        service_code = 'using AdcService = {};'.format(result['service_expr'].build(indent=0))
        adc_expr = TemplateExpr('AdcService::Adc', ['Context', 'Program', pins_expr])
        gen.add_global_resource(20, 'MyAdc', adc_expr, use_instance=True, code_before=service_code, context_name='Adc')
    
    gen.add_finalize_action(finalize)

def setup_pwm(gen, config, key):
    pwm_sel = selection.Selection()
    
    @pwm_sel.option('Disabled')
    def option(pwm_config):
        return None
    
    @pwm_sel.option('At91Sam3xPwm')
    def option(pwm_config):
        gen.add_aprinter_include('hal/at91/At91Sam3xPwm.h')
        
        return TemplateExpr('At91Sam3xPwm', ['Context', 'Program', TemplateExpr('At91Sam3xPwmParams', [
            pwm_config.get_int('PreA'),
            pwm_config.get_int('DivA'),
            pwm_config.get_int('PreB'),
            pwm_config.get_int('DivB'),
        ])])
    
    pwm_expr = config.do_selection(key, pwm_sel)
    if pwm_expr is not None:
        gen.add_global_resource(25, 'MyPwm', pwm_expr, context_name='Pwm')

def use_input_mode (config, key):
    im_sel = selection.Selection()
    
    @im_sel.option('At91SamPinInputMode')
    def option(im_config):
        return TemplateExpr('At91SamPinInputMode', [
            im_config.do_enum('PullMode', {'Normal': 'At91SamPinPullModeNormal', 'Pull-up': 'At91SamPinPullModePullUp'}),
        ])
    
    @im_sel.option('Stm32PinInputMode')
    def option(im_config):
        return TemplateExpr('Stm32PinInputMode', [
            im_config.do_enum('PullMode', {'Normal': 'Stm32PinPullModeNone', 'Pull-up': 'Stm32PinPullModePullUp', 'Pull-down': 'Stm32PinPullModePullDown'}),
        ])
    
    @im_sel.option('AvrPinInputMode')
    def option(im_config):
        return im_config.do_enum('PullMode', {'Normal': 'AvrPinInputModeNormal', 'Pull-up': 'AvrPinInputModePullUp'})
    
    @im_sel.option('Mk20PinInputMode')
    def option(im_config):
        return im_config.do_enum('PullMode', {'Normal': 'Mk20PinInputModeNormal', 'Pull-up': 'Mk20PinInputModePullUp', 'Pull-down': 'Mk20PinInputModePullDown'})
    
    @im_sel.option('StubPinInputMode')
    def option(im_config):
        return 'StubPinInputMode'
    
    return config.do_selection(key, im_sel)

def use_digital_input (gen, config, key):
    di = gen.get_object('digital_input', config, key)
    input_mode = use_input_mode(di, 'InputMode')
    return '{}, {}'.format(get_pin(gen, di, 'Pin'), _build_template_arg(input_mode, -1))

def use_analog_input (gen, config, key, user):
    ai = gen.get_object('analog_input', config, key)
    
    analog_input_sel = selection.Selection()
    
    @analog_input_sel.option('AdcAnalogInput')
    def option(analog_input):
        gen.add_aprinter_include('printer/analog_input/AdcAnalogInput.h')
        pin = get_pin(gen, analog_input, 'Pin')
        gen.get_singleton_object('adc_pins').append(pin)
        return TemplateExpr('AdcAnalogInputService', [pin])
    
    @analog_input_sel.option('Max31855AnalogInput')
    def option(analog_input):
        gen.add_aprinter_include('printer/analog_input/Max31855AnalogInput.h')
        return TemplateExpr('Max31855AnalogInputService', [
            get_pin(gen, analog_input, 'SsPin'),
            use_spi(gen, analog_input, 'SpiService', '{}::GetSpi'.format(user)),
        ])
    
    return ai.do_selection('Driver', analog_input_sel)

def use_interrupt_timer (gen, config, key, user, clearance=0.0):
    clock = gen.get_singleton_object('Clock')
    
    for it_config in config.enter_config(key):
        return clock.add_interrupt_timer(it_config.get_string('oc_unit'), user, clearance, it_config.path())

def use_pwm_output (gen, config, key, user, username, hard=False):
    pwm_output = gen.get_object('pwm_output', config, key)
    
    backend_sel = selection.Selection()
    
    @backend_sel.option('SoftPwm')
    def option(backend):
        if hard:
            config.path().error('Only Hard PWM is allowed here.')
        
        gen.add_aprinter_include('printer/pwm/SoftPwm.h')
        return TemplateExpr('SoftPwmService', [
            get_pin(gen, backend, 'OutputPin'),
            backend.get_bool('OutputInvert'),
            gen.add_float_constant('{}PulseInterval'.format(username), backend.get_float('PulseInterval')),
            use_interrupt_timer(gen, backend, 'Timer', '{}::TheTimer'.format(user))
        ])
    
    @backend_sel.option('HardPwm')
    def option(backend):
        gen.add_aprinter_include('printer/pwm/HardPwm.h')
        
        hard_driver_sel = selection.Selection()
        
        @hard_driver_sel.option('AvrClockPwm')
        def option(hard_driver):
            clock = gen.get_singleton_object('Clock')
            oc_unit = clock.check_oc_unit(hard_driver.get_string('oc_unit'), hard_driver.path())
            
            return TemplateExpr('AvrClockPwmService', [
                'AvrClockTcChannel{}{}'.format(oc_unit['tc'], oc_unit['channel']),
                get_pin(gen, hard_driver, 'OutputPin'),
            ])
        
        @hard_driver_sel.option('At91Sam3xPwmChannel')
        def option(hard_driver):
            return TemplateExpr('At91Sam3xPwmChannelService', [
                hard_driver.get_int('ChannelPrescaler'),
                hard_driver.get_int('ChannelPeriod'),
                hard_driver.get_int('ChannelNumber'),
                get_pin(gen, hard_driver, 'OutputPin'),
                TemplateChar(hard_driver.get_identifier('Signal')),
                hard_driver.get_bool('Invert'),
            ])
        
        hard_pwm_expr = backend.do_selection('HardPwmDriver', hard_driver_sel)
        
        if hard:
            return hard_pwm_expr
        
        return TemplateExpr('HardPwmService', [
            hard_pwm_expr
        ])
    
    return pwm_output.do_selection('Backend', backend_sel)

def use_spi (gen, config, key, user):
    spi_sel = selection.Selection()
    
    @spi_sel.option('At91SamSpi')
    def option(spi_config):
        gen.add_aprinter_include('hal/at91/At91SamSpi.h')
        devices = {
            'At91Sam3xSpiDevice':'AMBRO_AT91SAM3X_SPI_GLOBAL',
            'At91Sam3uSpiDevice':'AMBRO_AT91SAM3U_SPI_GLOBAL'
        }
        dev = spi_config.get_identifier('Device')
        if dev not in devices:
            spi_config.path().error('Incorrect SPI device.')
        gen.add_isr('{}({}, Context())'.format(devices[dev], user))
        return TemplateExpr('At91SamSpiService', [dev])
    
    @spi_sel.option('At91SamUsartSpi')
    def option(spi_config):
        gen.add_aprinter_include('hal/at91/At91SamUsartSpi.h')
        dev_index = spi_config.get_int('DeviceIndex')
        gen.add_isr('APRINTER_AT91SAM_USART_SPI_GLOBAL({}, {}, Context())'.format(dev_index, user))
        return TemplateExpr('At91SamUsartSpiService', [
            'At91SamUsartSpiDevice{}'.format(dev_index),
            spi_config.get_int('ClockDivider'),
        ])
    
    @spi_sel.option('AvrSpi')
    def option(spi_config):
        gen.add_aprinter_include('hal/avr/AvrSpi.h')
        gen.add_isr('AMBRO_AVR_SPI_ISRS({}, Context())'.format(user))
        return TemplateExpr('AvrSpiService', [spi_config.get_int('SpeedDiv')])
    
    return config.do_selection(key, spi_sel)

def use_sdio (gen, config, key, user):
    sdio_sel = selection.Selection()
    
    @sdio_sel.option('Stm32Sdio')
    def option(sdio_config):
        gen.add_aprinter_include('hal/stm32/Stm32Sdio.h')
        gen.add_isr('APRINTER_STM32_SDIO_GLOBAL({}, Context())'.format(user))
        return TemplateExpr('Stm32SdioService', [
            sdio_config.get_bool('IsWideMode'),
            sdio_config.get_int('DataTimeoutBusClocks'),
            sdio_config.get_int('SdClockPrescaler'),
        ])
    
    @sdio_sel.option('At91SamSdio')
    def option(sdio_config):
        gen.add_aprinter_include('hal/at91/At91SamSdio.h')
        return TemplateExpr('At91SamSdioService', [
            sdio_config.get_int('Slot'),
            sdio_config.get_bool('IsWideMode'),
            sdio_config.get_int('MaxIoDescriptors'),
        ])
    
    return config.do_selection(key, sdio_sel)

def use_i2c (gen, config, key, user, username):
    i2c_sel = selection.Selection()
    
    @i2c_sel.option('At91SamI2c')
    def option(i2c_config):
        gen.add_aprinter_include('hal/at91/At91SamI2c.h')
        devices = {
            'At91SamI2cDevice1':1,
        }
        dev = i2c_config.get_identifier('Device')
        if dev not in devices:
            i2c_config.path().error('Incorrect I2C device.')
        gen.add_isr('AMBRO_AT91SAM_I2C_GLOBAL({}, {}, Context())'.format(devices[dev], user))
        return 'At91SamI2cService<{}, {}, {}>'.format(
            dev,
            i2c_config.get_int('Ckdiv'),
            gen.add_float_constant('{}I2cFreq'.format(username), i2c_config.get_float('I2cFreq'))
        )
    
    return config.do_selection(key, i2c_sel)

def use_eeprom(gen, config, key, user):
    eeprom_sel = selection.Selection()
    
    @eeprom_sel.option('I2cEeprom')
    def option(eeprom):
        gen.add_aprinter_include('hal/generic/I2cEeprom.h')
        return TemplateExpr('I2cEepromService', [use_i2c(gen, eeprom, 'I2c', '{}::GetI2c'.format(user), 'ConfigEeprom'), eeprom.get_int('I2cAddr'), eeprom.get_int('Size'), eeprom.get_int('BlockSize'), gen.add_float_constant('ConfigEepromWriteTimeout', eeprom.get_float('WriteTimeout'))])
    
    @eeprom_sel.option('TeensyEeprom')
    def option(eeprom):
        gen.add_aprinter_include('hal/teensy3/TeensyEeprom.h')
        return TemplateExpr('TeensyEepromService', [eeprom.get_int('Size'), eeprom.get_int('FakeBlockSize')])
    
    @eeprom_sel.option('AvrEeprom')
    def option(eeprom):
        gen.add_aprinter_include('hal/avr/AvrEeprom.h')
        gen.add_isr('AMBRO_AVR_EEPROM_ISRS({}, Context())'.format(user))
        return TemplateExpr('AvrEepromService', [eeprom.get_int('FakeBlockSize')])
    
    @eeprom_sel.option('FlashWrapper')
    def option(eeprom):
        gen.add_aprinter_include('hal/generic/FlashWrapper.h')
        return TemplateExpr('FlashWrapperService', [
            use_flash(gen, eeprom, 'FlashDriver', '{}::GetFlash'.format(user)),
        ])
    
    return config.do_selection(key, eeprom_sel)

def use_flash(gen, config, key, user):
    flash_sel = selection.Selection()
    
    @flash_sel.option('At91SamFlash')
    def option(flash):
        device_index = flash.get_int('DeviceIndex')
        if not (0 <= device_index < 10):
            flash.key_path('DeviceIndex').error('Invalid device index.')
        gen.add_aprinter_include('hal/at91/At91Sam3xFlash.h')
        gen.add_isr('AMBRO_AT91SAM3X_FLASH_GLOBAL({}, {}, Context())'.format(device_index, user))
        return TemplateExpr('At91Sam3xFlashService', [
            'At91Sam3xFlashDevice{}'.format(device_index)
        ])
    
    return config.do_selection(key, flash_sel)

def use_serial(gen, config, key, user):
    serial_sel = selection.Selection()
    
    @serial_sel.option('AsfUsbSerial')
    def option(serial_service):
        gen.add_aprinter_include('hal/at91/AsfUsbSerial.h')
        gen.add_init_call(0, 'udc_start();')
        return 'AsfUsbSerialService'
    
    @serial_sel.option('At91Sam3xSerial')
    def option(serial_service):
        gen.add_aprinter_include('hal/at91/At91Sam3xSerial.h')
        gen.add_isr('AMBRO_AT91SAM3X_SERIAL_GLOBAL({}, Context())'.format(user))
        if serial_service.get_bool('UseForDebug'):
            gen.add_aprinter_include('hal/generic/NewlibDebugWrite.h')
            gen.add_global_code(0, 'APRINTER_SETUP_NEWLIB_DEBUG_WRITE(At91Sam3xSerial_DebugWrite<{}>, Context())'.format(user))
        return 'At91Sam3xSerialService'
    
    @serial_sel.option('TeensyUsbSerial')
    def option(serial_service):
        gen.add_aprinter_include('hal/teensy3/TeensyUsbSerial.h')
        gen.add_global_code(0, 'extern "C" { void usb_init (void); }')
        gen.add_init_call(0, 'usb_init();')
        return 'TeensyUsbSerialService'
    
    @serial_sel.option('AvrSerial')
    def option(serial_service):
        gen.add_aprinter_include('hal/avr/AvrSerial.h')
        gen.add_aprinter_include('hal/avr/AvrDebugWrite.h')
        gen.add_isr('AMBRO_AVR_SERIAL_ISRS({}, Context())'.format(user))
        gen.add_global_code(0, 'APRINTER_SETUP_AVR_DEBUG_WRITE(AvrSerial_DebugPutChar<{}>, Context())'.format(user))
        gen.add_init_call(-2, 'aprinter_init_avr_debug_write();')
        return TemplateExpr('AvrSerialService', [serial_service.get_bool('DoubleSpeed')])
    
    @serial_sel.option('Stm32UsbSerial')
    def option(serial_service):
        gen.add_aprinter_include('hal/stm32/Stm32UsbSerial.h')
        return 'Stm32UsbSerialService'
    
    @serial_sel.option('LinuxStdInOutSerial')
    def option(serial_service):
        gen.add_aprinter_include('hal/linux/LinuxStdInOutSerial.h')
        return 'LinuxStdInOutSerialService'
    
    @serial_sel.option('NullSerial')
    def option(serial_service):
        gen.add_aprinter_include('hal/generic/NullSerial.h')
        return 'NullSerialService'
    
    return config.do_selection(key, serial_sel)

def use_sdcard(gen, config, key, user):
    sd_service_sel = selection.Selection()

    @sd_service_sel.option('SpiSdCard')
    def option(spi_sd):
        gen.add_aprinter_include('hal/generic/SpiSdCard.h')
        return TemplateExpr('SpiSdCardService', [
            get_pin(gen, spi_sd, 'SsPin'),
            use_spi(gen, spi_sd, 'SpiService', '{}::GetSpi'.format(user)),
        ])
    
    @sd_service_sel.option('SdioSdCard')
    def option(sdio_sd):
        gen.add_aprinter_include('hal/generic/SdioSdCard.h')
        return TemplateExpr('SdioSdCardService', [
            use_sdio(gen, sdio_sd, 'SdioService', '{}::GetSdio'.format(user)),
        ])
    
    @sd_service_sel.option('LinuxSdCard')
    def option(linux_sd):
        gen.add_aprinter_include('hal/linux/LinuxSdCard.h')
        return TemplateExpr('LinuxSdCardService', [
            linux_sd.get_int('BlockSize'),
            linux_sd.get_int('MaxIoBlocks'),
            linux_sd.get_int('MaxIoDescriptors'),
        ])
    
    return config.do_selection(key, sd_service_sel)

def use_config_manager(gen, config, key, user):
    config_manager_sel = selection.Selection()
    
    @config_manager_sel.option('ConstantConfigManager')
    def option(config_manager):
        gen.add_aprinter_include('printer/config_manager/ConstantConfigManager.h')
        return 'ConstantConfigManagerService'
    
    @config_manager_sel.option('RuntimeConfigManager')
    def option(config_manager):
        gen.add_aprinter_include('printer/config_manager/RuntimeConfigManager.h')
        
        config_store_sel = selection.Selection()
        
        @config_store_sel.option('NoStore')
        def option(config_store):
            return 'RuntimeConfigManagerNoStoreService'
        
        @config_store_sel.option('EepromConfigStore')
        def option(config_store):
            gen.add_aprinter_include('printer/config_store/EepromConfigStore.h')
            
            return TemplateExpr('EepromConfigStoreService', [
                use_eeprom(gen, config_store, 'Eeprom', '{}::GetStore<>::GetEeprom'.format(user)),
                config_store.get_int('StartBlock'),
                config_store.get_int('EndBlock'),
            ])
        
        @config_store_sel.option('FileConfigStore')
        def option(config_store):
            gen.add_aprinter_include('printer/config_store/FileConfigStore.h')
            return 'FileConfigStoreService'
        
        return TemplateExpr('RuntimeConfigManagerService', [config_manager.do_selection('ConfigStore', config_store_sel)])
    
    return config.do_selection(key, config_manager_sel)

def use_microstep(gen, config, key):
    ms_driver_sel = selection.Selection()
    
    @ms_driver_sel.option('A4982')
    def option(ms_driver_config):
        gen.add_aprinter_include('printer/microstep/A4982MicroStep.h')
        
        return 'A4982MicroStepService<{}, {}>'.format(
            get_pin(gen, ms_driver_config, 'Ms1Pin'),
            get_pin(gen, ms_driver_config, 'Ms2Pin'),
        )
    
    @ms_driver_sel.option('A4988')
    def option(ms_driver_config):
        gen.add_aprinter_include('printer/microstep/A4988MicroStep.h')
        
        return 'A4988MicroStepService<{}, {}, {}>'.format(
            get_pin(gen, ms_driver_config, 'Ms1Pin'),
            get_pin(gen, ms_driver_config, 'Ms2Pin'),
            get_pin(gen, ms_driver_config, 'Ms3Pin'),
        )
    
    return config.do_selection(key, ms_driver_sel)

def use_current_driver(gen, config, key, user):
    current_driver_sel = selection.Selection()
    
    @current_driver_sel.option('Ad5206Current')
    def option(current_driver):
        gen.add_aprinter_include('printer/current/Ad5206Current.h')
        
        return TemplateExpr('Ad5206CurrentService', [
            get_pin(gen, current_driver, 'SsPin'),
            use_spi(gen, current_driver, 'SpiService', '{}::GetSpi'.format(user)),
        ])
    
    return config.do_selection(key, current_driver_sel)

def use_current_driver_channel(gen, config, key, name):
    current_driver_channel_sel = selection.Selection()
    
    @current_driver_channel_sel.option('Ad5206CurrentChannelParams')
    def option(current_driver_channel):
        return TemplateExpr('Ad5206CurrentChannelParams', [
            current_driver_channel.get_int('DevChannelIndex'),
            gen.add_float_config('{}CurrentConversionFactor'.format(name), current_driver_channel.get_float('ConversionFactor'))
        ])
    
    return config.do_selection(key, current_driver_channel_sel)

def get_letter_number_name(config, key):
    name = config.get_string(key)
    match = re.match('\\A([A-Z])([0-9]{0,2})\\Z', name)
    if not match:
        config.key_path(key).error('Incorrect name (expecting letter optionally followed by a number).')
    letter = match.group(1)
    number = int(match.group(2)) if match.group(2) != '' else 0
    normalized_name = '{}{}'.format(letter, number) if number != 0 else letter
    return normalized_name, TemplateExpr('AuxControlName', [TemplateChar(letter), number])

def get_ip_index(gen, config, key):
    index_name = config.get_string(key)
    if index_name not in ('MruListIndex', 'AvlTreeIndex'):
        config.key_path(key).error('Invalid value.')
    gen.add_include('aipstack/structure/index/{}.h'.format(index_name))
    return 'AIpStack::{}Service'.format(index_name)

def get_heap_structure(gen, config, key):
    structure_name = config.get_string(key)
    if structure_name not in ('LinkedHeap', 'SortedList'):
        config.key_path(key).error('Invalid value.')
    gen.add_aprinter_include('structure/{}.h'.format(structure_name))
    return 'APrinter::{}Service'.format(structure_name)

class NetworkConfigState(object):
    def __init__(self, min_send_buf, min_recv_buf):
        self.min_send_buf = min_send_buf
        self.min_recv_buf = min_recv_buf
        self._num_connections = 0
    
    def add_resource_counts(self, connections=0):
        self._num_connections += connections

def setup_network(gen, config, key, assertions_enabled):
    network_sel = selection.Selection()
    
    @network_sel.option('NoNetwork')
    def option(network_config):
        return False
    
    @network_sel.option('Network')
    def option(network_config):
        gen.add_extra_include_path('aipstack/src')
        gen.add_aprinter_include('net/IpStackNetwork.h')
        gen.add_include('aipstack/ip/IpReassembly.h')
        gen.add_include('aipstack/ip/IpPathMtuCache.h')
        
        num_arp_entries = network_config.get_int('NumArpEntries')
        if not 4 <= num_arp_entries <= 128:
            network_config.key_path('NumArpEntries').error('Value out of range.')
        
        arp_protect_count = network_config.get_int('ArpProtectCount')
        if not 2 <= arp_protect_count <= num_arp_entries:
            network_config.key_path('ArpProtectCount').error('Value out of range.')
        
        max_reass_packets = network_config.get_int('MaxReassPackets')
        if not 1 <= max_reass_packets <= 128:
            network_config.key_path('MaxReassPackets').error('Value out of range.')
        
        max_reass_size = network_config.get_int('MaxReassSize')
        if not 1480 <= max_reass_size <= 30000:
            network_config.key_path('MaxReassSize').error('Value out of range.')
        
        max_reass_holes = network_config.get_int('MaxReassHoles')
        if not 1 <= max_reass_holes <= 250:
            network_config.key_path('MaxReassHoles').error('Value out of range.')
        
        max_reass_time_sec = network_config.get_int('MaxReassTimeSeconds')
        if not 5 <= max_reass_time_sec <= 255:
            network_config.key_path('MaxReassTimeSeconds').error('Value out of range.')
        
        mtu_timeout_minutes = network_config.get_int('MtuTimeoutMinutes')
        if not 1 <= mtu_timeout_minutes <= 255:
            network_config.key_path('MtuTimeoutMinutes').error('Value out of range.')
        
        num_tcp_pcbs = network_config.get_int('NumTcpPcbs')
        if not 2 <= num_tcp_pcbs <= 2048:
            network_config.key_path('NumTcpPcbs').error('Value out of range.')
        
        num_oos_segs = network_config.get_int('NumOosSegs')
        if not 1 <= num_oos_segs <= 32:
            network_config.key_path('NumOosSegs').error('Value out of range.')
        
        tcp_wnd_upd_thr_div = network_config.get_int('TcpWndUpdThrDiv')
        
        link_with_array_indices = network_config.get_bool('LinkWithArrayIndices')
        
        checksum_src_file = gen.get_singleton_object('checksum_src_file', allow_none=True)
        if checksum_src_file is not None:
            gen.add_extra_source(checksum_src_file)
            gen.add_define('AIPSTACK_EXTERNAL_CHKSUM', 1)
        
        gen.add_define('AIPSTACK_CONFIG_ASSERT_INCLUDE', '<aprinter/base/Assert.h>')
        gen.add_define('AIPSTACK_CONFIG_ASSERT_HANDLER', 'APRINTER_AIPSTACK_ASSERT_HANDLER')
        
        if assertions_enabled:
            gen.add_define('AIPSTACK_CONFIG_ENABLE_ASSERTIONS')
        
        service_expr = TemplateExpr('IpStackNetworkService', [
            use_ethernet(gen, network_config, 'EthernetDriver', 'MyNetwork::GetEthernet'),
            num_arp_entries,
            arp_protect_count,
            TemplateExpr('AIpStack::IpPathMtuCacheService', [
                'AIpStack::IpPathMtuCacheOptions::NumMtuEntries::Is<{}>'.format('IpStackNumMtuEntries'),
                'AIpStack::IpPathMtuCacheOptions::MtuTimeoutMinutes::Is<{}>'.format(mtu_timeout_minutes),
                'AIpStack::IpPathMtuCacheOptions::MtuIndexService::Is<{}>'.format(
                    get_ip_index(gen, network_config, 'MtuIndexService')),
            ]),
            TemplateExpr('AIpStack::IpReassemblyService', [
                'AIpStack::IpReassemblyOptions::MaxReassEntrys::Is<{}>'.format(max_reass_packets),
                'AIpStack::IpReassemblyOptions::MaxReassSize::Is<{}>'.format(max_reass_size),
                'AIpStack::IpReassemblyOptions::MaxReassHoles::Is<{}>'.format(max_reass_holes),
                'AIpStack::IpReassemblyOptions::MaxReassTimeSeconds::Is<{}>'.format(max_reass_time_sec),
            ]),
            num_tcp_pcbs,
            num_oos_segs,
            tcp_wnd_upd_thr_div,
            get_ip_index(gen, network_config, 'PcbIndexService'),
            link_with_array_indices,
            get_heap_structure(gen, network_config, 'ArpTableTimersStructureService'),
        ])
        service_code = 'using NetworkService = {};'.format(service_expr.build(indent=0))
        network_expr = TemplateExpr('NetworkService::Compose', ['Context', 'Program'])
        gen.add_global_resource(27, 'MyNetwork', network_expr, use_instance=True, code_before=service_code, context_name='Network', is_fast_event_root=True)
        
        tcp_max_mss = 1460
        min_send_buf = 2*tcp_max_mss
        min_recv_buf = 2*tcp_max_mss
        
        network_state = NetworkConfigState(min_send_buf, min_recv_buf)
        gen.register_singleton_object('network', network_state)
        
        def finalize():
            num_mtu_entries = network_state._num_connections
            gen.add_int_constant('int', 'IpStackNumMtuEntries', num_mtu_entries)
        
        gen.add_finalize_action(finalize)
        
        return True
    
    return config.do_selection(key, network_sel)

def use_ethernet(gen, config, key, user):
    ethernet_sel = selection.Selection()
    
    @ethernet_sel.option('MiiEthernet')
    def option(ethernet_config):
        gen.add_aprinter_include('hal/generic/MiiEthernet.h')
        return TemplateExpr('MiiEthernetService', [
            use_mii(gen, ethernet_config, 'MiiDriver', '{}::GetMii'.format(user)),
            use_phy(gen, ethernet_config, 'PhyDriver'),
        ])
    
    @ethernet_sel.option('LinuxTapEthernet')
    def option(ethernet_config):
        gen.add_aprinter_include('hal/linux/LinuxTapEthernet.h')
        return 'LinuxTapEthernetService'
    
    return config.do_selection(key, ethernet_sel)

def use_mii(gen, config, key, user):
    mii_sel = selection.Selection()
    
    @mii_sel.option('At91SamEmacMii')
    def option(mii_config):
        num_rx_buffers = mii_config.get_int('NumRxBufers')
        if not 12 <= num_rx_buffers <= 240:
            mii_config.key_path('NumRxBufers').error('Value out of range.')
        
        num_tx_buffers = mii_config.get_int('NumTxBufers')
        if not 1 <= num_tx_buffers <= 20:
            mii_config.key_path('NumTxBufers').error('Value out of range.')
        
        gen.add_aprinter_include('hal/at91/At91SamEmacMii.h')
        gen.add_extra_source('${ASF_DIR}/sam/drivers/emac/emac.c')
        gen.add_isr('APRINTER_AT91SAM_EMAC_MII_GLOBAL({}, Context())'.format(user))
        gen.add_define('APRINTER_AT91SAM_EMAC_NUM_RX_BUFFERS', num_rx_buffers)
        gen.add_define('APRINTER_AT91SAM_EMAC_NUM_TX_BUFFERS', num_tx_buffers)
        return 'At91SamEmacMiiService'
    
    return config.do_selection(key, mii_sel)

def use_phy(gen, config, key):
    phy_sel = selection.Selection()
    
    @phy_sel.option('GenericPhy')
    def option(phy_config):
        gen.add_aprinter_include('hal/ethernet_phy/GenericPhy.h')
        return TemplateExpr('GenericPhyService', [
            phy_config.get_bool('Rmii'),
            phy_config.get_int('PhyAddr'),
        ])
    
    return config.do_selection(key, phy_sel)

def generate(config_root_data, cfg_name, main_template):
    gen = GenState()
    
    for config_root in config_reader.start(config_root_data, config_reader_class=GenConfigReader):
        if cfg_name is None:
            cfg_name = config_root.get_string('selected_config')
        
        for config in config_root.enter_elem_by_id('configurations', 'name', cfg_name):
            board_name = config.get_string('board')
            
            aux_control_module = gen.add_module()
            aux_control_module_user = 'MyPrinter::GetModule<{}>'.format(aux_control_module.index)
            
            for board_data in config_root.enter_elem_by_id('boards', 'name', board_name):
                for platform_config in board_data.enter_config('platform_config'):
                    board_for_build = platform_config.get_string('board_for_build')
                    if not re.match('\\A[a-zA-Z0-9_]{1,128}\\Z', board_for_build):
                        platform_config.key_path('board_for_build').error('Incorrect format.')
                    gen.add_subst('BoardForBuild', board_for_build)
                    
                    output_types = []
                    for output_types_config in platform_config.enter_config('output_types'):
                        if output_types_config.get_bool('output_elf'):
                            output_types.append('elf')
                        if output_types_config.get_bool('output_bin'):
                            output_types.append('bin')
                        if output_types_config.get_bool('output_hex'):
                            output_types.append('hex')
                    
                    setup_platform(gen, platform_config, 'platform')
                    
                    for platform in platform_config.enter_config('platform'):
                        setup_clock(gen, platform, 'clock', clock_name='Clock', priority=-10, allow_disabled=False)
                        setup_pins(gen, platform, 'pins')
                        setup_adc(gen, platform, 'adc')
                        if platform.has('pwm'):
                            setup_pwm(gen, platform, 'pwm')
                        if platform.has('fast_clock'):
                            setup_clock(gen, platform, 'fast_clock', clock_name='FastClock', priority=-12, allow_disabled=True)
                        if platform.has('millisecond_clock'):
                            setup_millisecond_clock(gen, platform, 'millisecond_clock', priority=-13)
                    
                    setup_debug_interface(gen, platform_config, 'debug_interface')
                    
                    for helper_name in platform_config.get_list(config_reader.ConfigTypeString(), 'board_helper_includes', max_count=20):
                        if not re.match('\\A[a-zA-Z0-9_]{1,128}\\Z', helper_name):
                            platform_config.key_path('board_helper_includes').error('Invalid helper name.')
                        gen.add_aprinter_include('board/{}.h'.format(helper_name))
                
                gen.register_objects('digital_input', board_data, 'digital_inputs')
                gen.register_objects('stepper_port', board_data, 'stepper_ports')
                gen.register_objects('analog_input', board_data, 'analog_inputs')
                gen.register_objects('pwm_output', board_data, 'pwm_outputs')
                gen.register_objects('laser_port', board_data, 'laser_ports')
                
                led_pin_expr = get_pin(gen, board_data, 'LedPin')
                
                for performance in board_data.enter_config('performance'):
                    gen.add_typedef('TheAxisDriverPrecisionParams', performance.get_identifier('AxisDriverPrecisionParams'))
                    event_channel_timer_clearance = performance.get_float('EventChannelTimerClearance')
                    optimize_for_size = performance.get_bool('OptimizeForSize')
                    optimize_libc_for_size = performance.get_bool('OptimizeLibcForSize')
                
                event_channel_timer_expr = use_interrupt_timer(gen, board_data, 'EventChannelTimer', user='{}::GetEventChannelTimer<>'.format(aux_control_module_user), clearance=event_channel_timer_clearance)
                
                for development in board_data.enter_config('development'):
                    assertions_enabled = development.get_bool('AssertionsEnabled')
                    event_loop_benchmark_enabled = development.get_bool('EventLoopBenchmarkEnabled')
                    detect_overload_enabled = development.get_bool('DetectOverloadEnabled')
                    watchdog_debug_mode = development.get_bool('WatchdogDebugMode') if development.has('WatchdogDebugMode') else False
                    build_with_clang = development.get_bool('BuildWithClang')
                    verbose_build = development.get_bool('VerboseBuild')
                    debug_symbols = development.get_bool('DebugSymbols')
                    
                    if assertions_enabled:
                        gen.add_define('AMBROLIB_ASSERTIONS')
                    
                    if event_loop_benchmark_enabled:
                        gen.add_define('EVENTLOOP_BENCHMARK')
                    
                    if detect_overload_enabled:
                        gen.add_define('AXISDRIVER_DETECT_OVERLOAD')
                    
                    if development.get_bool('EnableBulkOutputTest'):
                        gen.add_aprinter_include('printer/modules/BulkOutputTestModule.h')
                        bulk_output_test_module = gen.add_module()
                        bulk_output_test_module.set_expr('BulkOutputTestModuleService')
                    
                    if development.get_bool('EnableBasicTestModule'):
                        gen.add_aprinter_include('printer/modules/BasicTestModule.h')
                        basic_test_module = gen.add_module()
                        basic_test_module.set_expr('BasicTestModuleService')
                    elif detect_overload_enabled:
                        development.key_path('DetectOverloadEnabled').error('BasicTestModule is required for overload detection.')
                    
                    if development.get_bool('EnableStubCommandModule'):
                        gen.add_aprinter_include('printer/modules/StubCommandModule.h')
                        stub_command_module = gen.add_module()
                        stub_command_module.set_expr('StubCommandModuleService')
                
                for serial in board_data.iter_list_config('serial_ports', max_count=5):
                    gen.add_aprinter_include('printer/modules/SerialModule.h')
                    gen.add_aprinter_include('printer/utils/GcodeParser.h')
                    
                    serial_module = gen.add_module()
                    serial_user = 'MyPrinter::GetModule<{}>::GetSerial'.format(serial_module.index)
                    
                    serial_module.set_expr(TemplateExpr('SerialModuleService', [
                        'UINT32_C({})'.format(serial.get_int_constant('BaudRate')),
                        serial.get_int_constant('RecvBufferSizeExp'),
                        serial.get_int_constant('SendBufferSizeExp'),
                        TemplateExpr('SerialGcodeParserService', [
                            serial.get_int_constant('GcodeMaxParts'),
                        ]),
                        use_serial(gen, serial, 'Service', serial_user),
                    ]))
                
                sdcard_sel = selection.Selection()
                
                @sdcard_sel.option('NoSdCard')
                def option(sdcard):
                    pass
                
                @sdcard_sel.option('SdCard')
                def option(sdcard):
                    gen.add_aprinter_include('printer/modules/SdCardModule.h')
                    
                    sdcard_module = gen.add_module()
                    sdcard_user = 'MyPrinter::GetModule<{}>::GetInput::GetSdCard'.format(sdcard_module.index)
                    
                    gcode_parser_sel = selection.Selection()
                    
                    @gcode_parser_sel.option('TextGcodeParser')
                    def option(parser):
                        gen.add_aprinter_include('printer/utils/GcodeParser.h')
                        return TemplateExpr('FileGcodeParserService', [
                            parser.get_int('MaxParts'),
                        ])
                    
                    @gcode_parser_sel.option('BinaryGcodeParser')
                    def option(parser):
                        gen.add_aprinter_include('printer/utils/BinaryGcodeParser.h')
                        return TemplateExpr('BinaryGcodeParserService', [
                            parser.get_int('MaxParts'),
                        ])
                    
                    fs_sel = selection.Selection()
                    
                    @fs_sel.option('Raw')
                    def option(fs_config):
                        gen.add_aprinter_include('printer/input/SdRawInput.h')
                        return TemplateExpr('SdRawInputService', [
                            use_sdcard(gen, sdcard, 'SdCardService', sdcard_user),
                        ])
                    
                    @fs_sel.option('Fat32')
                    def option(fs_config):
                        max_filename_size = fs_config.get_int('MaxFileNameSize')
                        if not (12 <= max_filename_size <= 1024):
                            fs_config.key_path('MaxFileNameSize').error('Bad value.')
                        
                        num_cache_entries = fs_config.get_int('NumCacheEntries')
                        if not (1 <= num_cache_entries <= 64):
                            fs_config.key_path('NumCacheEntries').error('Bad value.')
                        
                        max_io_blocks = fs_config.get_int('MaxIoBlocks')
                        if not (1 <= max_io_blocks <= num_cache_entries):
                            fs_config.key_path('MaxIoBlocks').error('Bad value.')
                        
                        gen.add_aprinter_include('printer/input/SdFatInput.h')
                        gen.add_aprinter_include('fs/FatFs.h')
                        
                        if fs_config.get_bool('EnableFsTest'):
                            gen.add_aprinter_include('printer/modules/FsTestModule.h')
                            fs_test_module = gen.add_module()
                            fs_test_module.set_expr('FsTestModuleService')
                        
                        gcode_upload_sel = selection.Selection()
                        
                        @gcode_upload_sel.option('NoGcodeUpload')
                        def option(gcode_upload_config):
                            pass
                        
                        @gcode_upload_sel.option('GcodeUpload')
                        def option(gcode_upload_config):
                            gen.add_aprinter_include('printer/modules/GcodeUploadModule.h')
                            
                            max_command_size = gcode_upload_config.get_int('MaxCommandSize')
                            if not (32 <= max_command_size <= 1024):
                                gcode_upload_config.key_path('MaxCommandSize').error('Bad value.')
                            
                            gcode_upload_module = gen.add_module()
                            gcode_upload_module.set_expr(TemplateExpr('GcodeUploadModuleService', [
                                max_command_size,
                            ]))
                        
                        fs_config.do_selection('GcodeUpload', gcode_upload_sel)
                        
                        return TemplateExpr('SdFatInputService', [
                            use_sdcard(gen, sdcard, 'SdCardService', sdcard_user),
                            TemplateExpr('FatFsService', [
                                max_filename_size,
                                num_cache_entries,
                                1, # NumIoUnits
                                max_io_blocks,
                                fs_config.get_bool_constant('CaseInsensFileName'),
                                fs_config.get_bool_constant('FsWritable'),
                                fs_config.get_bool_constant('EnableReadHinting'),
                            ]),
                            fs_config.get_bool_constant('HaveAccessInterface'),
                        ])
                    
                    sdcard_module.set_expr(TemplateExpr('SdCardModuleService', [
                        sdcard.do_selection('FsType', fs_sel),
                        sdcard.do_selection('GcodeParser', gcode_parser_sel),
                        sdcard.get_int('BufferBaseSize'),
                        sdcard.get_int('MaxCommandSize'),
                    ]))
                
                board_data.get_config('sdcard_config').do_selection('sdcard', sdcard_sel)
                
                config_manager_expr = use_config_manager(gen, board_data.get_config('runtime_config'), 'config_manager', 'MyPrinter::GetConfigManager')
                
                have_network = setup_network(gen, board_data.get_config('network_config'), 'network', assertions_enabled)
                if have_network:
                    network = gen.get_singleton_object('network')
                    
                    for network_config in board_data.get_config('network_config').enter_config('network'):
                        gen.add_aprinter_include('printer/modules/NetworkSupportModule.h')
                        network_support_module = gen.add_module()
                        network_support_module.set_expr(TemplateExpr('NetworkSupportModuleService', [
                            gen.add_bool_config('NetworkEnabled', network_config.get_bool('NetEnabled')),
                            gen.add_mac_addr_config('NetworkMacAddress', network_config.get_mac_addr('MacAddress')),
                            gen.add_bool_config('NetworkDhcpEnabled', network_config.get_bool('DhcpEnabled')),
                            gen.add_ip_addr_config('NetworkIpAddress', network_config.get_ip_addr('IpAddress')),
                            gen.add_ip_addr_config('NetworkIpNetmask', network_config.get_ip_addr('IpNetmask')),
                            gen.add_ip_addr_config('NetworkIpGateway', network_config.get_ip_addr('IpGateway')),
                        ]))
                        
                        tcpconsole_sel = selection.Selection()
                        
                        @tcpconsole_sel.option('NoTcpConsole')
                        def option(tcpconsole_config):
                            pass
                        
                        @tcpconsole_sel.option('TcpConsole')
                        def option(tcpconsole_config):
                            console_port = tcpconsole_config.get_int('Port')
                            if not (1 <= console_port <= 65534):
                                tcpconsole_config.key_path('Port').error('Bad value.')
                            
                            console_max_clients = tcpconsole_config.get_int('MaxClients')
                            if not (1 <= console_max_clients <= 32):
                                tcpconsole_config.key_path('MaxClients').error('Bad value.')
                            
                            console_max_pcbs = tcpconsole_config.get_int('MaxPcbs')
                            if not (console_max_clients <= console_max_pcbs):
                                tcpconsole_config.key_path('MaxPcbs').error('Bad value.')
                            
                            console_max_parts = tcpconsole_config.get_int('MaxParts')
                            if not (1 <= console_max_parts <= 64):
                                tcpconsole_config.key_path('MaxParts').error('Bad value.')
                            
                            console_max_command_size = tcpconsole_config.get_int('MaxCommandSize')
                            if not (1 <= console_max_command_size <= 512):
                                tcpconsole_config.key_path('MaxCommandSize').error('Bad value.')
                            
                            console_send_buf_size = tcpconsole_config.get_int('SendBufferSize')
                            if console_send_buf_size < network.min_send_buf:
                                tcpconsole_config.key_path('SendBufferSize').error('Bad value.')
                            
                            console_recv_buf_size = tcpconsole_config.get_int('RecvBufferSize')
                            if console_recv_buf_size < network.min_recv_buf:
                                tcpconsole_config.key_path('RecvBufferSize').error('Bad value.')
                            
                            gen.add_aprinter_include('printer/modules/TcpConsoleModule.h')
                            gen.add_aprinter_include('printer/utils/GcodeParser.h')
                            
                            tcp_console_module = gen.add_module()
                            tcp_console_module.set_expr(TemplateExpr('TcpConsoleModuleService', [
                                TemplateExpr('SerialGcodeParserService', [
                                    console_max_parts,
                                ]),
                                console_port,
                                console_max_clients,
                                console_max_pcbs,
                                console_max_command_size,
                                console_send_buf_size,
                                console_recv_buf_size,
                                gen.add_float_constant('TcpConsoleSendBufTimeout', tcpconsole_config.get_float('SendBufTimeout')),
                                gen.add_float_constant('TcpConsoleSendEndTimeout', tcpconsole_config.get_float('SendEndTimeout')),
                            ]))
                            
                            network.add_resource_counts(connections=console_max_clients)
                        
                        network_config.do_selection('tcpconsole', tcpconsole_sel)
                        
                        webif_sel = selection.Selection()
                        
                        @webif_sel.option('NoWebInterface')
                        def option(webif_config):
                            pass
                        
                        @webif_sel.option('WebInterface')
                        def option(webif_config):
                            webif_port = webif_config.get_int('Port')
                            if not (1 <= webif_port <= 65534):
                                webif_config.key_path('Port').error('Bad value.')
                            
                            webif_max_clients = webif_config.get_int('MaxClients')
                            if not (1 <= webif_max_clients <= 128):
                                webif_config.key_path('MaxClients').error('Bad value.')
                            
                            webif_queue_size = webif_config.get_int('QueueSize')
                            if not (0 <= webif_queue_size <= 512):
                                webif_config.key_path('QueueSize').error('Bad value.')
                            
                            webif_queue_recv_buffer_size = webif_config.get_int('QueueRecvBufferSize')
                            if not (0 < webif_queue_recv_buffer_size):
                                webif_config.key_path('QueueRecvBufferSize').error('Bad value.')
                            
                            webif_max_pcbs = webif_config.get_int('MaxPcbs')
                            if not (webif_max_clients+webif_queue_size <= webif_max_pcbs):
                                webif_config.key_path('MaxPcbs').error('Bad value.')
                            
                            webif_send_buf_size = webif_config.get_int('SendBufferSize')
                            if webif_send_buf_size < network.min_send_buf:
                                webif_config.key_path('SendBufferSize').error('Bad value.')
                            
                            webif_recv_buf_size = webif_config.get_int('RecvBufferSize')
                            if webif_recv_buf_size < network.min_recv_buf:
                                webif_config.key_path('RecvBufferSize').error('Bad value (too small).')
                            if webif_recv_buf_size < webif_queue_recv_buffer_size:
                                webif_config.key_path('RecvBufferSize').error('Bad value (less than QueueRecvBufferSize).')
                            
                            allow_persistent = webif_config.get_bool('AllowPersistent')
                            
                            gen.add_float_constant('WebInterfaceQueueTimeout', webif_config.get_float('QueueTimeout'))
                            gen.add_float_constant('WebInterfaceInactivityTimeout', webif_config.get_float('InactivityTimeout'))
                            
                            if webif_config.get_bool('EnableDebug'):
                                gen.add_define('APRINTER_DEBUG_HTTP_SERVER', 1)
                            
                            gen.add_aprinter_include('printer/modules/WebInterfaceModule.h')
                            gen.add_aprinter_include('printer/utils/GcodeParser.h')
                            
                            # Add modules with request handlers before the WebInterfaceModule
                            # so we have a clean deinit path - first active requests in a module
                            # are deinited, then the associated module is deinited.
                            
                            if True:
                                gen.add_aprinter_include('printer/modules/WebApiFilesModule.h')
                                config_web_api_module = gen.add_module()
                                config_web_api_module.set_expr('WebApiFilesModuleService')
                            
                            if config_manager_expr != 'ConstantConfigManagerService':
                                gen.add_aprinter_include('printer/modules/WebApiConfigModule.h')
                                config_web_api_module = gen.add_module()
                                config_web_api_module.set_expr('WebApiConfigModuleService')
                            
                            webif_module = gen.add_module()
                            webif_module.set_expr(TemplateExpr('WebInterfaceModuleService', [
                                TemplateExpr('HttpServerNetParams', [
                                    webif_port,
                                    webif_max_clients,
                                    webif_queue_size,
                                    webif_queue_recv_buffer_size,
                                    webif_max_pcbs,
                                    webif_send_buf_size,
                                    webif_recv_buf_size,
                                    allow_persistent,
                                    'WebInterfaceQueueTimeout',
                                    'WebInterfaceInactivityTimeout',
                                ]),
                                webif_config.get_int('JsonBufferSize'),
                                webif_config.get_int('NumGcodeSlots'),
                                TemplateExpr('SerialGcodeParserService', [
                                    webif_config.get_int('MaxGcodeParts'),
                                ]),
                                webif_config.get_int('MaxGcodeCommandSize'),
                                gen.add_float_constant('WebInterfaceGcodeSendBufTimeout', webif_config.get_float('GcodeSendBufTimeout')),
                            ]))
                            
                            network.add_resource_counts(connections=webif_max_clients+webif_queue_size)
                            
                        network_config.do_selection('webinterface', webif_sel)
                        
                        for development in board_data.enter_config('development'):
                            networktest_sel = selection.Selection()
                            
                            @networktest_sel.option('Disabled')
                            def option(networksel_config):
                                pass
                            
                            @networktest_sel.option('Enabled')
                            def option(networksel_config):
                                gen.add_aprinter_include('printer/modules/NetworkTestModule.h')
                                network_test_module = gen.add_module()
                                network_test_module.set_expr(TemplateExpr('NetworkTestModuleService', [
                                    networksel_config.get_int('BufferSize'),
                                ]))
                                network.add_resource_counts(connections=1)
                            
                            development.do_selection('NetworkTestModule', networktest_sel)
                
                current_config = board_data.get_config('current_config')
            
            gen.add_aprinter_include('printer/PrinterMain.h')
            gen.add_float_constant('FanSpeedMultiply', 1.0 / 255.0)
            
            for advanced in config.enter_config('advanced'):
                gen.add_float_constant('LedBlinkInterval', advanced.get_float('LedBlinkInterval'))
                gen.add_float_config('ForceTimeout', advanced.get_float('ForceTimeout'))
            
            current_control_channel_list = []
            microstep_axis_list = []
            
            def stepper_cb(stepper, stepper_index):
                name = stepper.get_id_char('Name')
                
                homing_sel = selection.Selection()
                
                @homing_sel.option('no_homing')
                def option(homing):
                    return 'PrinterMainNoHomingParams'
                
                @homing_sel.option('homing')
                def option(homing):
                    gen.add_aprinter_include('printer/utils/AxisHomer.h')
                    
                    return TemplateExpr('PrinterMainHomingParams', [
                        gen.add_bool_config('{}HomeDir'.format(name), homing.get_bool('HomeDir')),
                        gen.add_float_config('{}HomeOffset'.format(name), homing.get_float('HomeOffset')),
                        TemplateExpr('AxisHomerService', [
                            use_digital_input(gen, homing, 'HomeEndstopInput'),
                            gen.add_bool_config('{}HomeEndInvert'.format(name), homing.get_bool('HomeEndInvert')),
                            gen.add_float_config('{}HomeFastMaxDist'.format(name), homing.get_float('HomeFastMaxDist')),
                            gen.add_float_config('{}HomeRetractDist'.format(name), homing.get_float('HomeRetractDist')),
                            gen.add_float_config('{}HomeSlowMaxDist'.format(name), homing.get_float('HomeSlowMaxDist')),
                            gen.add_float_config('{}HomeFastSpeed'.format(name), homing.get_float('HomeFastSpeed')),
                            gen.add_float_config('{}HomeRetractSpeed'.format(name), homing.get_float('HomeRetractSpeed')),
                            gen.add_float_config('{}HomeSlowSpeed'.format(name), homing.get_float('HomeSlowSpeed')),
                        ])
                    ])
                
                gen.add_aprinter_include('printer/actuators/AxisDriver.h')
                
                stepper_ports_for_axis = []
                
                def slave_steppers_cb(slave_stepper, slave_stepper_index):
                    slave_stepper_port = gen.get_object('stepper_port', slave_stepper, 'stepper_port')
                    stepper_ports_for_axis.append(slave_stepper_port)
                    
                    stepper_config_prefix = name if slave_stepper_index == 0 else '{}S{}'.format(name, slave_stepper_index)
                    
                    stepper_current_sel = selection.Selection()
                    
                    @stepper_current_sel.option('NoCurrent')
                    def option(stepper_current_config):
                        pass
                    
                    @stepper_current_sel.option('Current')
                    def option(stepper_current_config):
                        stepper_current_expr = TemplateExpr('MotorCurrentAxisParams', [
                            TemplateChar(name), # M906 will only work on all slave steppers of an axis...
                            gen.add_float_config('{}Current'.format(stepper_config_prefix), slave_stepper.get_float('Current')),
                            use_current_driver_channel(gen, stepper_current_config, 'DriverChannelParams', stepper_config_prefix),
                        ])
                        current_control_channel_list.append(stepper_current_expr)
                    
                    slave_stepper_port.do_selection('current', stepper_current_sel)
                    
                    microstep_sel = selection.Selection()
                    
                    @microstep_sel.option('NoMicroStep')
                    def option(microstep_config):
                        pass
                    
                    @microstep_sel.option('MicroStep')
                    def option(microstep_config):
                        microstep_expr = TemplateExpr('MicroStepAxisParams', [
                            use_microstep(gen, microstep_config, 'MicroStepDriver'),
                            slave_stepper.get_int('MicroSteps'),
                        ])
                        microstep_axis_list.append(microstep_expr)
                    
                    slave_stepper_port.do_selection('microstep', microstep_sel)
                    
                    return TemplateExpr('PrinterMainSlaveStepperParams', [
                        TemplateExpr('StepperDef', [
                            get_pin(gen, slave_stepper_port, 'DirPin'),
                            get_pin(gen, slave_stepper_port, 'StepPin'),
                            get_pin(gen, slave_stepper_port, 'EnablePin'),
                            slave_stepper_port.get_bool('StepLevel'),
                            slave_stepper_port.get_bool('EnableLevel'),
                            gen.add_bool_config('{}InvertDir'.format(stepper_config_prefix), slave_stepper.get_bool('InvertDir')),
                        ]),
                    ])
                
                slave_steppers_expr = stepper.do_list('slave_steppers', slave_steppers_cb, min_count=1, max_count=10)
                
                delay_sel = selection.Selection()
                
                @delay_sel.option('NoDelay')
                def option(delay_config):
                    return 'AxisDriverNoDelayParams'
                
                @delay_sel.option('Delay')
                def option(delay_config):
                    return TemplateExpr('AxisDriverDelayParams', [
                        gen.add_float_constant('{}DirSetTime'.format(name), delay_config.get_float('DirSetTime')),
                        gen.add_float_constant('{}StepHighTime'.format(name), delay_config.get_float('StepHighTime')),
                        gen.add_float_constant('{}StepLowTime'.format(name), delay_config.get_float('StepLowTime')),
                    ])
                
                first_stepper_port = stepper_ports_for_axis[0]
                if first_stepper_port.get_config('StepperTimer').get_string('_compoundName') != 'interrupt_timer':
                    first_stepper_port.key_path('StepperTimer').error('Stepper port of first stepper in axis must have a timer unit defined.')
                
                return TemplateExpr('PrinterMainAxisParams', [
                    TemplateChar(name),
                    gen.add_float_config('{}StepsPerUnit'.format(name), stepper.get_float('StepsPerUnit')),
                    gen.add_float_config('{}MinPos'.format(name), stepper.get_float('MinPos')),
                    gen.add_float_config('{}MaxPos'.format(name), stepper.get_float('MaxPos')),
                    gen.add_float_config('{}MaxSpeed'.format(name), stepper.get_float('MaxSpeed')),
                    gen.add_float_config('{}MaxAccel'.format(name), stepper.get_float('MaxAccel')),
                    gen.add_float_config('{}DistanceFactor'.format(name), stepper.get_float('DistanceFactor')),
                    gen.add_float_config('{}CorneringDistance'.format(name), stepper.get_float('CorneringDistance')),
                    stepper.do_selection('homing', homing_sel),
                    stepper.get_bool('EnableCartesianSpeedLimit'),
                    stepper.get_bool('IsExtruder'),
                    32,
                    TemplateExpr('AxisDriverService', [
                        use_interrupt_timer(gen, first_stepper_port, 'StepperTimer', user='MyPrinter::GetAxisTimer<{}>'.format(stepper_index)),
                        'TheAxisDriverPrecisionParams',
                        stepper.get_bool('PreloadCommands'),
                        stepper.do_selection('delay', delay_sel),
                    ]),
                    slave_steppers_expr,
                ])
            
            steppers_expr = config.do_list('steppers', stepper_cb, min_count=1, max_count=15)
            
            def heater_cb(heater, heater_index):
                name, name_expr = get_letter_number_name(heater, 'Name')
                
                conversion_sel = selection.Selection()
                
                @conversion_sel.option('conversion')
                def option(conversion_config):
                    gen.add_aprinter_include('printer/thermistor/GenericThermistor.h')
                    return TemplateExpr('GenericThermistorService', [
                        gen.add_float_config('{}HeaterTempResistorR'.format(name), conversion_config.get_float('ResistorR')),
                        gen.add_float_config('{}HeaterTempR0'.format(name), conversion_config.get_float('R0')),
                        gen.add_float_config('{}HeaterTempBeta'.format(name), conversion_config.get_float('Beta')),
                        gen.add_float_config('{}HeaterTempMinTemp'.format(name), conversion_config.get_float('MinTemp')),
                        gen.add_float_config('{}HeaterTempMaxTemp'.format(name), conversion_config.get_float('MaxTemp')),
                    ])
                
                @conversion_sel.option('PtRtdFormula')
                def option(conversion_config):
                    gen.add_aprinter_include('printer/thermistor/PtRtdFormula.h')
                    return TemplateExpr('PtRtdFormulaService', [
                        gen.add_float_config('{}HeaterTempResistorR'.format(name), conversion_config.get_float('ResistorR')),
                        gen.add_float_config('{}HeaterTempPtR0'.format(name), conversion_config.get_float('PtR0')),
                        gen.add_float_config('{}HeaterTempPtA'.format(name), conversion_config.get_float('PtA')),
                        gen.add_float_config('{}HeaterTempPtB'.format(name), conversion_config.get_float('PtB')),
                        gen.add_float_config('{}HeaterTempMinTemp'.format(name), conversion_config.get_float('MinTemp')),
                        gen.add_float_config('{}HeaterTempMaxTemp'.format(name), conversion_config.get_float('MaxTemp')),
                    ])
                
                @conversion_sel.option('Max31855Formula')
                def option(conversion_config):
                    gen.add_aprinter_include('printer/thermistor/Max31855Formula.h')
                    return 'Max31855FormulaService'
                
                @conversion_sel.option('E3dPt100')
                def option(conversion_config):
                    gen.add_aprinter_include('printer/thermistor/InterpolationTableThermistor_tables.h')
                    return TemplateExpr('InterpolationTableThermistorService', ['InterpolationTableE3dPt100'])
                
                conversion = heater.do_selection('conversion', conversion_sel)
                
                for control in heater.enter_config('control'):
                    gen.add_aprinter_include('printer/temp_control/PidControl.h')
                    control_interval = control.get_float('ControlInterval')
                    control_service = TemplateExpr('PidControlService', [
                        gen.add_float_config('{}HeaterPidP'.format(name), control.get_float('PidP')),
                        gen.add_float_config('{}HeaterPidI'.format(name), control.get_float('PidI')),
                        gen.add_float_config('{}HeaterPidD'.format(name), control.get_float('PidD')),
                        gen.add_float_config('{}HeaterPidIStateMin'.format(name), control.get_float('PidIStateMin')),
                        gen.add_float_config('{}HeaterPidIStateMax'.format(name), control.get_float('PidIStateMax')),
                        gen.add_float_config('{}HeaterPidDHistory'.format(name), control.get_float('PidDHistory')),
                    ])
                
                for observer in heater.enter_config('observer'):
                    gen.add_aprinter_include('printer/utils/TemperatureObserver.h')
                    observer_service = TemplateExpr('TemperatureObserverService', [
                        gen.add_float_config('{}HeaterObserverInterval'.format(name), observer.get_float('ObserverInterval')),
                        gen.add_float_config('{}HeaterObserverTolerance'.format(name), observer.get_float('ObserverTolerance')),
                        gen.add_float_config('{}HeaterObserverMinTime'.format(name), observer.get_float('ObserverMinTime')),
                    ])
                
                cold_extrusion_sel = selection.Selection()
                
                @cold_extrusion_sel.option('NoColdExtrusionPrevention')
                def option(cold_extrusion_config):
                    return 'AuxControlNoColdExtrusionParams'
                
                @cold_extrusion_sel.option('ColdExtrusionPrevention')
                def option(cold_extrusion_config):
                    extruders_exprs = []
                    for axis_name in cold_extrusion_config.get_list(config_reader.ConfigTypeString(), 'ExtruderAxes', max_count=20):
                        extruders_exprs.append(TemplateExpr('WrapInt', [TemplateChar(axis_name)]))
                    
                    return TemplateExpr('AuxControlColdExtrusionParams', [
                        gen.add_float_config('{}HeaterMinExtrusionTemp'.format(name), cold_extrusion_config.get_float('MinExtrusionTemp')),
                        TemplateList(extruders_exprs),
                    ])
                
                cold_extrusion = heater.do_selection('cold_extrusion_prevention', cold_extrusion_sel)
                
                return TemplateExpr('AuxControlModuleHeaterParams', [
                    name_expr,
                    heater.get_int('SetMCommand'),
                    use_analog_input(gen, heater, 'ThermistorInput', '{}::GetHeaterAnalogInput<{}>'.format(aux_control_module_user, heater_index)),
                    conversion,
                    gen.add_float_config('{}HeaterMinSafeTemp'.format(name), heater.get_float('MinSafeTemp')),
                    gen.add_float_config('{}HeaterMaxSafeTemp'.format(name), heater.get_float('MaxSafeTemp')),
                    gen.add_float_config('{}HeaterControlInterval'.format(name), control_interval),
                    control_service,
                    observer_service,
                    use_pwm_output(gen, heater, 'pwm_output', '{}::GetHeaterPwm<{}>'.format(aux_control_module_user, heater_index), '{}Heater'.format(name)),
                    cold_extrusion,
                ])
            
            heaters_expr = config.do_list('heaters', heater_cb, max_count=15)
            
            transform_sel = selection.Selection()
            transform_axes = []
            
            @transform_sel.option('NoTransform')
            def option(transform):
                return 'PrinterMainNoTransformParams'
            
            @transform_sel.default()
            def default(transform_type, transform):
                virt_homing_axes = []
                
                def virtual_axis_cb(virtual_axis, virtual_axis_index):
                    name = virtual_axis.get_id_char('Name')
                    transform_axes.append(name)
                    
                    homing_sel = selection.Selection()
                    
                    @homing_sel.option('no_homing')
                    def option(homing):
                        pass
                    
                    @homing_sel.option('homing')
                    def option(homing):
                        virt_homing_axes.append(TemplateExpr('VirtualHomingModuleAxisParams', [
                            TemplateChar(name),
                            gen.add_bool_config('{}HomeByDefault'.format(name), homing.get_bool('ByDefault')),
                            use_digital_input(gen, homing, 'HomeEndstopInput'),
                            gen.add_bool_config('{}HomeEndInvert'.format(name), homing.get_bool('HomeEndInvert')),
                            gen.add_bool_config('{}HomeDir'.format(name), homing.get_bool('HomeDir')),
                            gen.add_float_config('{}HomeFastExtraDist'.format(name), homing.get_float('HomeFastExtraDist')),
                            gen.add_float_config('{}HomeRetractDist'.format(name), homing.get_float('HomeRetractDist')),
                            gen.add_float_config('{}HomeSlowExtraDist'.format(name), homing.get_float('HomeSlowExtraDist')),
                            gen.add_float_config('{}HomeFastSpeed'.format(name), homing.get_float('HomeFastSpeed')),
                            gen.add_float_config('{}HomeRetractSpeed'.format(name), homing.get_float('HomeRetractSpeed')),
                            gen.add_float_config('{}HomeSlowSpeed'.format(name), homing.get_float('HomeSlowSpeed')),
                        ]))
                    
                    virtual_axis.do_selection('homing', homing_sel)
                    
                    return TemplateExpr('PrinterMainVirtualAxisParams', [
                        TemplateChar(name),
                        gen.add_float_config('{}MinPos'.format(name), virtual_axis.get_float('MinPos')),
                        gen.add_float_config('{}MaxPos'.format(name), virtual_axis.get_float('MaxPos')),
                        gen.add_float_config('{}MaxSpeed'.format(name), virtual_axis.get_float('MaxSpeed')),
                    ])
                
                def transform_stepper_cb(transform_stepper, transform_stepper_index):
                    stepper_name = transform_stepper.get_id_char('StepperName')
                    try:
                        stepper_generator = config.enter_elem_by_id('steppers', 'Name', stepper_name)
                    except config_reader.ConfigError:
                        transform_stepper.path().error('Unknown stepper \'{}\' referenced.'.format(stepper_name))
                    
                    for stepper in stepper_generator:
                        if stepper.get_bool('EnableCartesianSpeedLimit'):
                            stepper.key_path('EnableCartesianSpeedLimit').error('Stepper involved coordinate transform may not be cartesian.')
                    
                    return TemplateExpr('WrapInt', [TemplateChar(stepper_name)])
                
                transform_type_sel = selection.Selection()
                
                @transform_type_sel.option('Null')
                def option():
                    gen.add_aprinter_include('printer/transform/IdentityTransform.h')
                    return TemplateExpr('IdentityTransformService', [0]), 'Transform'
                
                @transform_type_sel.option('CoreXY')
                def option():
                    gen.add_aprinter_include('printer/transform/CoreXyTransform.h')
                    return 'CoreXyTransformService', 'Transform'
                
                @transform_type_sel.option('Delta')
                def option():
                    gen.add_aprinter_include('printer/transform/DeltaTransform.h')
                    return TemplateExpr('DeltaTransformService', [
                        gen.add_float_config('DeltaDiagonalRod', transform.get_float('DiagnalRod')),
                        gen.add_float_config('DeltaSmoothRodOffset', transform.get_float('SmoothRodOffset')),
                        gen.add_float_config('DeltaEffectorOffset', transform.get_float('EffectorOffset')),
                        gen.add_float_config('DeltaCarriageOffset', transform.get_float('CarriageOffset')),
                        gen.add_float_config('DeltaLimitRadius', transform.get_float('LimitRadius')),
                    ]), 'Delta'
                
                @transform_type_sel.option('RotationalDelta')
                def option():
                    gen.add_aprinter_include('printer/transform/RotationalDeltaTransform.h')
                    return TemplateExpr('RotationalDeltaTransformService', [
                        gen.add_float_config('DeltaEndEffectorLength', transform.get_float('EndEffectorLength')),
                        gen.add_float_config('DeltaBaseLength', transform.get_float('BaseLength')),
                        gen.add_float_config('DeltaRodLength', transform.get_float('RodLength')),
                        gen.add_float_config('DeltaArmLength', transform.get_float('ArmLength')),
                        gen.add_float_config('DeltaZOffset', transform.get_float('ZOffset')),
                    ]), 'Delta'
                
                @transform_type_sel.option('SCARA')
                def option():
                    gen.add_aprinter_include('printer/transform/SCARATransform.h')
                    return TemplateExpr('SCARATransformService', [
                        gen.add_float_config('SCARAArm1Length', transform.get_float('Arm1Length')),
                        gen.add_float_config('SCARAArm2Length', transform.get_float('Arm2Length')),
                        gen.add_bool_config('SCARAExternalArm2Motor', transform.get_bool('ExternalArm2Motor')),
                        gen.add_float_config('SCARAXOffset', transform.get_float('XOffset')),
                        gen.add_float_config('SCARAYOffset', transform.get_float('YOffset')),
                    ]), 'SCARA'
                
                @transform_type_sel.option('DualSCARA')
                def option():
                    gen.add_aprinter_include('printer/transform/DualSCARATransform.h')
                    return TemplateExpr('DualSCARATransformService', [
                        gen.add_float_config('SCARAArm1ShoulderXCoord', transform.get_float('Arm1ShoulderXCoord')),
                        gen.add_float_config('SCARAArm2ShoulderXCoord', transform.get_float('Arm2ShoulderXCoord')),
                        gen.add_float_config('SCARAArm1ProximalSideLength', transform.get_float('Arm1ProximalSideLength')),
                        gen.add_float_config('SCARAArm2ProximalSideLength', transform.get_float('Arm2ProximalSideLength')),
                        gen.add_float_config('SCARAArm1DistalSideLength', transform.get_float('Arm1DistalSideLength')),
                        gen.add_float_config('SCARAArm2DistalSideLength', transform.get_float('Arm2DistalSideLength')),
                        gen.add_float_config('SCARAXOffset', transform.get_float('XOffset')),
                        gen.add_float_config('SCARAYOffset', transform.get_float('YOffset')),
                    ]), 'SCARA'
                
                transform_type_expr, transform_prefix = transform_type_sel.run(transform_type)
                
                splitter_sel = selection.Selection()
                
                @splitter_sel.option('NoSplitter')
                def option(splitter):
                    gen.add_aprinter_include('printer/transform/NoSplitter.h')
                    return 'NoSplitterService'
                
                @splitter_sel.option('DistanceSplitter')
                def option(splitter):
                    gen.add_aprinter_include('printer/transform/DistanceSplitter.h')
                    return TemplateExpr('DistanceSplitterService', [
                        gen.add_float_config('{}MinSplitLength'.format(transform_prefix), splitter.get_float('MinSplitLength')),
                        gen.add_float_config('{}MaxSplitLength'.format(transform_prefix), splitter.get_float('MaxSplitLength')),
                        gen.add_float_config('{}SegmentsPerSecond'.format(transform_prefix), splitter.get_float('SegmentsPerSecond')),
                    ])
                
                splitter_expr = transform.do_selection('Splitter', splitter_sel)
                
                max_dimensions = 10
                
                dimension_count = transform.get_int('DimensionCount')
                if not 0 <= dimension_count <= max_dimensions:
                    transform.path().error('Incorrect DimensionCount.')
                
                virtual_axes = transform.do_keyed_list(dimension_count, 'CartesianAxes', 'VirtualAxis', virtual_axis_cb)
                transform_steppers = transform.do_keyed_list(dimension_count, 'Steppers', 'TransformStepper', transform_stepper_cb)
                
                if transform.has('IdentityAxes'):
                    num_idaxes = 0
                    
                    for idaxis in transform.iter_list_config('IdentityAxes', max_count=max_dimensions-dimension_count):
                        virt_axis_name = idaxis.get_id_char('Name')
                        stepper_name = idaxis.get_id_char('StepperName')
                        
                        limits_sel = selection.Selection()
                        
                        @limits_sel.option('LimitsAsStepper')
                        def option(limits_config):
                            return [
                                '{}MinPos'.format(stepper_name),
                                '{}MaxPos'.format(stepper_name),
                            ]
                        
                        @limits_sel.option('LimitsSpecified')
                        def option(limits_config):
                            return [
                                gen.add_float_config('{}MinPos'.format(virt_axis_name), limits_config.get_float('MinPos')),
                                gen.add_float_config('{}MaxPos'.format(virt_axis_name), limits_config.get_float('MaxPos')),
                            ]
                        
                        limits = idaxis.do_selection('Limits', limits_sel)
                        
                        transform_axes.append(virt_axis_name)
                        num_idaxes += 1
                        
                        virtual_axes.append_arg(TemplateExpr('PrinterMainVirtualAxisParams', [
                            TemplateChar(virt_axis_name),
                            limits[0],
                            limits[1],
                            gen.add_float_config('{}MaxSpeed'.format(virt_axis_name), float('inf'), is_constant=True),
                        ]))
                        
                        transform_steppers.append_arg(TemplateExpr('WrapInt', [TemplateChar(stepper_name)]))
                    
                    if num_idaxes > 0:
                        gen.add_aprinter_include('printer/transform/IdentityTransform.h')
                        id_transform_type_expr = TemplateExpr('IdentityTransformService', [num_idaxes])
                        
                        if dimension_count == 0:
                            transform_type_expr = id_transform_type_expr
                        else:
                            gen.add_aprinter_include('printer/transform/CombineTransform.h')
                            transform_type_expr = TemplateExpr('CombineTransformService', [TemplateList([transform_type_expr, id_transform_type_expr])])
                        
                        dimension_count += num_idaxes
                
                if dimension_count == 0:
                    transform.path().error('Need at least one dimension.')
                
                if len(virt_homing_axes) > 0:
                    gen.add_aprinter_include('printer/modules/VirtualHomingModule.h')
                    virt_homing_module = gen.add_module()
                    virt_homing_module.set_expr(TemplateExpr('VirtualHomingModuleService', [
                        TemplateList(virt_homing_axes),
                    ]))
                
                return TemplateExpr('PrinterMainTransformParams', [
                    virtual_axes,
                    transform_steppers,
                    transform_type_expr,
                    splitter_expr,
                ])
            
            transform_expr = config.do_selection('transform', transform_sel)
            
            probe_sel = selection.Selection()
            
            @probe_sel.option('NoProbe')
            def option(probe):
                return False
            
            @probe_sel.option('Probe')
            def option(probe):
                gen.add_aprinter_include('printer/modules/BedProbeModule.h')
                
                probe_module = gen.add_module()
                
                gen.add_bool_config('ProbeInvert', probe.get_bool('InvertInput')),
                gen.add_float_config('ProbeOffsetX', probe.get_float('OffsetX'))
                gen.add_float_config('ProbeOffsetY', probe.get_float('OffsetY'))
                gen.add_float_config('ProbeStartHeight', probe.get_float('StartHeight'))
                gen.add_float_config('ProbeLowHeight', probe.get_float('LowHeight'))
                gen.add_float_config('ProbeRetractDist', probe.get_float('RetractDist'))
                gen.add_float_config('ProbeMoveSpeed', probe.get_float('MoveSpeed'))
                gen.add_float_config('ProbeFastSpeed', probe.get_float('FastSpeed'))
                gen.add_float_config('ProbeRetractSpeed', probe.get_float('RetractSpeed'))
                gen.add_float_config('ProbeSlowSpeed', probe.get_float('SlowSpeed'))
                gen.add_float_config('ProbeGeneralZOffset', probe.get_float('GeneralZOffset'))
                
                num_points = 0
                for (i, point) in enumerate(probe.iter_list_config('ProbePoints', min_count=1, max_count=20)):
                    num_points += 1
                    gen.add_bool_config('ProbeP{}Enabled'.format(i+1), point.get_bool('Enabled'))
                    gen.add_float_config('ProbeP{}X'.format(i+1), point.get_float('X'))
                    gen.add_float_config('ProbeP{}Y'.format(i+1), point.get_float('Y'))
                    gen.add_float_config('ProbeP{}ZOffset'.format(i+1), point.get_float('Z-offset'))
                
                correction_sel = selection.Selection()
                
                @correction_sel.option('NoCorrection')
                def option(correction):
                    return 'BedProbeNoCorrectionParams'
                
                @correction_sel.option('Correction')
                def option(correction):
                    if 'Z' not in transform_axes:
                        correction.path().error('Bed correction is only supported when the Z axis is involved in the coordinate transformation.')
                    
                    quadratic_supported = correction.get_bool('QuadraticCorrectionSupported')
                    quadratic_enabled = gen.add_bool_config('ProbeQuadrCorrEnabled', correction.get_bool('QuadraticCorrectionEnabled')) if quadratic_supported else 'void'
                    
                    return TemplateExpr('BedProbeCorrectionParams', [quadratic_supported, quadratic_enabled])
                
                correction_expr = probe.do_selection('correction', correction_sel)
                
                probe_module.set_expr(TemplateExpr('BedProbeModuleService', [
                    'MakeTypeList<WrapInt<\'X\'>, WrapInt<\'Y\'>>',
                    '\'Z\'',
                    use_digital_input(gen, probe, 'ProbePin'),
                    'ProbeInvert',
                    'MakeTypeList<ProbeOffsetX, ProbeOffsetY>',
                    'ProbeStartHeight',
                    'ProbeLowHeight',
                    'ProbeRetractDist',
                    'ProbeMoveSpeed',
                    'ProbeFastSpeed',
                    'ProbeRetractSpeed',
                    'ProbeSlowSpeed',
                    'ProbeGeneralZOffset',
                    TemplateList(['BedProbePointParams<ProbeP{0}Enabled, MakeTypeList<ProbeP{0}X, ProbeP{0}Y>, ProbeP{0}ZOffset>'.format(i+1) for i in range(num_points)]),
                    correction_expr,
                ]))
                
                return True
            
            have_bed_probing = config.get_config('probe_config').do_selection('probe', probe_sel)
            
            def fan_cb(fan, fan_index):
                name, name_expr = get_letter_number_name(fan, 'Name')
                
                return TemplateExpr('AuxControlModuleFanParams', [
                    name_expr,
                    fan.get_int('SetMCommand'),
                    fan.get_int('OffMCommand'),
                    'FanSpeedMultiply',
                    use_pwm_output(gen, fan, 'pwm_output', '{}::GetFanPwm<{}>'.format(aux_control_module_user, fan_index), '{}Fan'.format(name))
                ])
            
            fans_expr = config.do_list('fans', fan_cb, max_count=15)
            
            def laser_cb(laser, laser_index):
                gen.add_aprinter_include('printer/actuators/LaserDriver.h')
                gen.add_aprinter_include('printer/duty_formula/LinearDutyFormula.h')
                
                name = laser.get_id_char('Name')
                laser_port = gen.get_object('laser_port', laser, 'laser_port')
                
                return TemplateExpr('PrinterMainLaserParams', [
                    TemplateChar(name),
                    TemplateChar(laser.get_id_char('DensityName')),
                    gen.add_float_config('{}LaserPower'.format(name), laser.get_float('LaserPower')),
                    gen.add_float_config('{}MaxPower'.format(name), laser.get_float('MaxPower')),
                    use_pwm_output(gen, laser_port, 'pwm_output', '', '', hard=True),
                    TemplateExpr('LinearDutyFormulaService', [15]),
                    TemplateExpr('LaserDriverService', [
                        use_interrupt_timer(gen, laser_port, 'LaserTimer', user='MyPrinter::GetLaserDriver<{}>::TheTimer'.format(laser_index)),
                        gen.add_float_constant('{}AdjustmentInterval'.format(name), laser.get_float('AdjustmentInterval')),
                        'LaserDriverDefaultPrecisionParams',
                    ]),
                ])
            
            lasers_expr = config.do_list('lasers', laser_cb, max_count=15)
            
            current_sel = selection.Selection()
            
            @current_sel.option('NoCurrent')
            def option(current):
                pass
            
            @current_sel.option('Current')
            def option(current):
                gen.add_aprinter_include('printer/modules/MotorCurrentModule.h')
                current_module = gen.add_module()
                current_module.set_expr(TemplateExpr('MotorCurrentModuleService', [
                    TemplateList(current_control_channel_list),
                    use_current_driver(gen, current, 'current_driver', 'MyPrinter::GetModule<{}>::GetDriver'.format(current_module.index))
                ]))
            
            current_config.do_selection('current', current_sel)
            
            if len(microstep_axis_list) > 0:
                gen.add_aprinter_include('printer/modules/MicroStepConfigModule.h')
                microstep_module = gen.add_module()
                microstep_module.set_expr(TemplateExpr('MicroStepConfigModuleService', [
                    TemplateList(microstep_axis_list),
                ]))
            
            gen.add_aprinter_include('printer/modules/AuxControlModule.h')
            aux_control_module.set_expr(TemplateExpr('AuxControlModuleService', [
                performance.get_int_constant('EventChannelBufferSize'),
                event_channel_timer_expr,
                gen.add_float_config('WaitTimeout', config.get_float('WaitTimeout')),
                gen.add_float_config('WaitReportPeriod', config.get_float('WaitReportPeriod')),
                heaters_expr,
                fans_expr,
            ]))
            
            moves_sel = selection.Selection()
            
            @moves_sel.option('NoMoves')
            def option(moves_config):
                pass
            
            @moves_sel.option('Moves')
            def option(moves_config):
                gen.add_aprinter_include('printer/modules/MoveToModule.h')
                
                def move_cb(move_config, move_index):
                    move_config_prefix = 'Move{}'.format(move_index+1)
                    move_coords = set()
                    
                    def coord_cb(coord_config, coord_index):
                        axis_name = coord_config.get_id_char('AxisName')
                        if axis_name in move_coords:
                            coord_config.key_path('AxisName').error('Duplicate axis in coordinate.')
                        move_coords.add(axis_name)
                        
                        return TemplateExpr('MoveCoordSpec', [
                            TemplateChar(axis_name),
                            gen.add_float_config('{}{}'.format(move_config_prefix, axis_name), coord_config.get_float('Value')),
                        ])
                    
                    hook_type = move_config.do_enum('HookType', {
                        'After homing':      'ServiceList::AfterDefaultHomingHookService',
                        'After bed probing': 'ServiceList::AfterBedProbingHookService',
                    })
                    
                    if hook_type == 'ServiceList::AfterBedProbingHookService' and not have_bed_probing:
                        move_config.key_path('HookType').error('Cannot use bed probing hook without bed probing configured.')
                    
                    return TemplateExpr('MoveSpec', [
                        hook_type,
                        move_config.get_int('HookPriority'),
                        gen.add_bool_config('{}Enabled'.format(move_config_prefix), move_config.get_bool('Enabled')),
                        gen.add_float_config('{}Speed'.format(move_config_prefix), move_config.get_float('Speed')),
                        move_config.do_list('Coordinates', coord_cb, max_count=10),
                    ])
                
                moveto_module = gen.add_module()
                moveto_module.set_expr(TemplateExpr('MoveToModuleService', [
                    moves_config.do_list('Moves', move_cb, max_count=10),
                ]))
            
            config.do_selection('Moves', moves_sel)
            
            if gen._need_millisecond_clock:
                if not gen._have_hw_millisecond_clock:
                    gen.add_aprinter_include('system/MillisecondClock.h')
                    gen.add_global_resource(5, 'MyMillisecondClock', TemplateExpr('MillisecondClock', ['Context', 'Program']), context_name='MillisecondClock')
                
                gen.add_aprinter_include('printer/modules/MillisecondClockInfoModule.h')
                millisecond_clock_module = gen.add_module()
                millisecond_clock_module.set_expr('MillisecondClockInfoModuleService')
            
            printer_params = TemplateExpr('PrinterMainParams', [
                led_pin_expr,
                'LedBlinkInterval',
                gen.add_float_config('InactiveTime', config.get_float('InactiveTime')),
                performance.get_int_constant('ExpectedResponseLength'),
                performance.get_int_constant('ExtraSendBufClearance'),
                performance.get_int_constant('MaxMsgSize'),
                gen.add_float_constant('SpeedLimitMultiply', 1.0 / 60.0),
                gen.add_float_config('MaxStepsPerCycle', performance.get_float('MaxStepsPerCycle')),
                performance.get_int_constant('StepperSegmentBufferSize'),
                performance.get_int_constant('LookaheadBufferSize'),
                performance.get_int_constant('LookaheadCommitCount'),
                'ForceTimeout',
                performance.get_identifier('FpType', lambda x: x in ('float', 'double')),
                setup_watchdog(gen, platform, 'watchdog', 'MyPrinter::GetWatchdog'),
                watchdog_debug_mode,
                config_manager_expr,
                'ConfigList',
                steppers_expr,
                transform_expr,
                lasers_expr,
                TemplateList(gen._modules_exprs),
            ])
            
            printer_params_typedef = 'struct ThePrinterParams : public {} {{}};'.format(printer_params.build(0))
            printer_expr = TemplateExpr('PrinterMainArg', ['Context', 'Program', 'ThePrinterParams'])
            
            gen.add_global_resource(30, 'MyPrinter', printer_expr, use_instance=True, context_name='Printer', code_before=printer_params_typedef, is_fast_event_root=True)
            gen.add_subst('EmergencyProvider', 'MyPrinter')
            
            setup_event_loop(gen)
    
    gen.finalize()
    
    return {
        'main_source': rich_template.RichTemplate(main_template).substitute(gen.get_subst()),
        'board_for_build': board_for_build,
        'output_types': output_types,
        'optimize_for_size': optimize_for_size,
        'optimize_libc_for_size': optimize_libc_for_size,
        'build_with_clang': build_with_clang,
        'verbose_build': verbose_build,
        'debug_symbols': debug_symbols,
        'build_vars': gen._build_vars,
        'extra_sources': gen._extra_sources,
        'extra_include_paths': gen._extra_include_paths,
        'defines': gen._defines,
        'linker_symbols': gen._linker_symbols,
    }

def main():
    # Parse arguments.
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--config', help='JSON configuration file to use.')
    parser.add_argument('--cfg-name', help='Build this configuration instead of the one specified in the configuration file.')
    parser.add_argument('--output', default='-', help='File to write the output to (C++ code or Nix expression).')
    parser.add_argument('--system', help='Pass system to nixpkgs (build on that platform).')
    args = parser.parse_args()
    
    # Determine directories.
    src_dir = file_utils.file_dir(__file__)
    nix_dir = os.path.join(src_dir, '..', '..', 'nix')
    
    # Read the configuration.
    config = args.config if args.config is not None else os.path.join(src_dir, '..', 'gui', 'default_config.json')
    with file_utils.use_input_file(config) as config_f:
        config_data = json.load(config_f)
    
    # Read main template file.
    main_template = file_utils.read_file(os.path.join(src_dir, 'main_template.cpp'))
    
    # Call the generate function.
    result = generate(config_data, args.cfg_name, main_template)
    
    # Build the Nix expression.
    nix_expr = (
        'with ((import (builtins.toPath {})) {{ pkgs = import <nixpkgs> {{ {} }}; }}); aprinterFunc {{\n'
        '    boardName = {}; buildName = "aprinter"; desiredOutputs = {}; optimizeForSize = {};\n'
        '    optimizeLibcForSize = {};\n'
        '    buildWithClang = {}; verboseBuild = {}; debugSymbols = {}; buildVars = {};\n'
        '    extraSources = {}; extraIncludePaths = {}; defines = {}; linkerSymbols = {};\n'
        '    mainText = {};\n'
        '}}\n'
    ).format(
        nix_utils.escape_string_for_nix(nix_dir),
        '' if args.system is None else 'system = {};'.format(nix_utils.escape_string_for_nix(args.system)),
        nix_utils.escape_string_for_nix(result['board_for_build']),
        nix_utils.convert_for_nix(result['output_types']),
        nix_utils.convert_bool_for_nix(result['optimize_for_size']),
        nix_utils.convert_bool_for_nix(result['optimize_libc_for_size']),
        nix_utils.convert_bool_for_nix(result['build_with_clang']),
        nix_utils.convert_bool_for_nix(result['verbose_build']),
        nix_utils.convert_bool_for_nix(result['debug_symbols']),
        nix_utils.convert_for_nix(result['build_vars']),
        nix_utils.convert_for_nix(result['extra_sources']),
        nix_utils.convert_for_nix(result['extra_include_paths']),
        nix_utils.convert_for_nix(result['defines']),
        nix_utils.convert_for_nix(result['linker_symbols']),
        nix_utils.escape_string_for_nix(result['main_source'])
    )
    
    # Write output.
    with file_utils.use_output_file(args.output) as output_f:
        output_f.write(nix_expr)

main()
