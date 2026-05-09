#!/bin/bash
# 临时禁用用户本地包
mv ~/.local/lib/python3.10/site-packages ~/.local/lib/python3.10/site-packages.backup 2>/dev/null || true

# 运行脚本
/usr/bin/python3.10 "$@" 

# 恢复用户本地包
mv ~/.local/lib/python3.10/site-packages.backup ~/.local/lib/python3.10/site-packages 2>/dev/null || true
