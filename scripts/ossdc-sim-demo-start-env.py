#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import print_function
from __future__ import unicode_literals

import pexpect
import sys
import time
import os
from environs import Env

debugging = True

def force_exit(err):
    try:
        sys.exit(err)
    except SystemExit:
        os._exit(err)

def run_cmd(parent,egoNr,cmdStr):
    adeName = "ade"+str(egoNr)
    shellCmd = 'sh -c "~/adehome/ade --name ade'+str(egoNr)+' enter"'
    print('Start shell:',shellCmd)
    child = pexpect.spawn(shellCmd)
    if debugging:  # Just in case you also want to see the output for debugging
        child.logfile = open(adeName+".log", "ab") #sys.stdout
    
    COMMAND_PROMPT = "Entering "+adeName
    i = child.expect([pexpect.TIMEOUT, COMMAND_PROMPT ],10)
    if i != 0: 
        print('Start cmd:',cmdStr)
        child.sendline(cmdStr)
    else:
        print('Timeout on:',cmdStr)
    return child


childProc = []

env = Env()

MULTI_EGO_COUNT = 2

mainProc = []
cmdStrList = []
for i in range(MULTI_EGO_COUNT):
    egoNr = i
    cmdStrList.append('sh -c "EGO_NR='+str(egoNr)+' ~/adehome/ade --name ade'+str(egoNr)+' --rc ../OSSDC-SIM-Demos/ade-envs/aderc-gpu-multi-foxy start --update"')
    print('Start cmd:',cmdStrList[i])
    mainProc.append(pexpect.spawn(cmdStrList[i]))
    # mainProc1.logfile = sys.stdout

    childProc.append(mainProc[i])

expectResult = []
oneFailed = False
for i in range(MULTI_EGO_COUNT):
    egoNr = i
    COMMAND_PROMPT = "ADE has been started"
    expectResult.append(mainProc[i].expect([pexpect.TIMEOUT, COMMAND_PROMPT ],120))
    if expectResult[i] == 0: 
        print('Timeout on:',cmdStrList[i])
        oneFaied = True

if not oneFailed:
    for i in range(MULTI_EGO_COUNT):

        egoNr = i
        
        time.sleep(1)

        cmdStr = 'lgsvl_bridge'
        child = run_cmd(mainProc[i],egoNr,cmdStr)
        childProc.append(child)

        cmdStr = 'cd ~/rosboard && ./run'
        child = run_cmd(mainProc[i],egoNr,cmdStr)
        childProc.append(child)

x=input("Click ENTER to STOP the run")

for i in range(MULTI_EGO_COUNT):
    egoNr = i
    pexpect.spawn('ade --name ade'+str(egoNr)+' stop')

time.sleep(2)

for child in childProc:
    if child.isalive():
        child.close()
    # Print the final state of the child. Normally isalive() should be FALSE.
    if child.isalive():
        print('Child did not exit gracefully.')
    else:
        print('Child exited gracefully.')
