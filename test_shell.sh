#!/bin/bash
echo "test" > /mnt/e/puppyfangzhen/test_echo.txt
python3 -c "print('hello from python')" > /mnt/e/puppyfangzhen/test_py.txt 2>&1
ls -la /mnt/e/puppyfangzhen/test_*.txt
