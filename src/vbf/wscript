## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    module = bld.create_ns3_module('vbf', ['core','internet', 'wifi','uan'])
    module.includes = '.'
    module.source = [
        'model/vbf-packet.cc',
        'model/vbf-routing-protocol.cc',
        'helper/vbf-helper.cc',
        ]

    # vbf_test = bld.create_ns3_module_test_library('vbf')
    # vbf_test.source = [
    #     ]

    headers = bld(features='ns3header')
    headers.module = 'vbf'
    headers.source = [
        'model/vbf-packet.h',
        'model/vbf-routing-protocol.h',
        'helper/vbf-helper.h',
        ]

    # if bld.env['ENABLE_EXAMPLES']:
    #     bld.recurse('examples')

    # bld.ns3_python_bindings()
