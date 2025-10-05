def ToHex(obj): # Convert to hex only if signal is longer than 16 bits
    binary_str = str(obj)
    binary_str = binary_str.strip()
    if(len(binary_str)>=16  and  binary_str.replace("1","").replace("0","") == ""): # Convert to hex only if value is longer than 16 bits, and doesn't contain 'x' or 'z' bits.
        value = int(binary_str,2)
        hex_len = (len(binary_str)+3)//4
        hex_str = format(value, '0{}x'.format(hex_len))
        return "0x"+hex_str
    else:
        return binary_str

def Log_Everything(dut, instance, log_submodules=False):
    # This functions scans a module instance, and prints values of every signal it finds
    instance_name = instance.name
    wires = []
    submodules = []
    for attribute_name in dir(instance):
        attribute = getattr(instance, attribute_name)
        if attribute.__class__.__module__.startswith('cocotb.handle'):
            if(attribute.__class__.__name__ == 'ModifiableObject'):        # wire / reg
                wires.append((attribute_name, ToHex(attribute.value)) )
            elif(attribute.__class__.__name__ == 'NonHierarchyIndexableObject'):  # wire / reg array
                wires.append((attribute_name, [ToHex(v) for v in attribute.value] ) )
            elif(attribute.__class__.__name__ == 'HierarchyObject'):       # submodule
                submodules.append((attribute_name, attribute.get_definition_name()) )
            elif(attribute.__class__.__name__ == 'HierarchyArrayObject'):  # submodule array
                submodules.append((attribute_name, f"[{len(attribute)}]") )
    
    if(log_submodules):
        for sub in submodules:
            dut._log.debug(f"{instance_name}.{sub[0]:<20} is {sub[1]}")
    for wire in wires:
        dut._log.debug(f"{instance_name}.{wire[0]:<20} = {wire[1]}")


#Populate the below functions as in the example lines of code to print your values for debugging
def Log_Datapath(dut,logger):
    #Log whatever signal you want from the datapath, called before positive clock edge
    logger.debug("************ DUT DATAPATH Signals ***************")
    dut._log.info("RegWrite: %s", ToHex(dut.dp.RegWriteW))
    Log_Everything(dut, dut.dp)


def Log_Controller(dut,logger):
    #Log whatever signal you want from the controller, called before positive clock edge
    logger.debug("************ DUT Controller Signals ***************")
    dut._log.info("execute: %s", ToHex(dut.ctrl.and_3))

    dut._log.info("decode: %s", ToHex(dut.ctrl.RegWriteD))
    dut._log.info("memory: %s", ToHex(dut.ctrl.RegWriteM_r))
    dut._log.info("wb: %s", ToHex(dut.ctrl.RegWriteW_r))
    Log_Everything(dut, dut.ctrl)
