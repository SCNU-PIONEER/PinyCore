# .gdbinit
set p obj on
set p pretty on


python
import os
import sys
import platform
# 添加libstdc++打印脚本路径（根据实际路径修改）
if platform.system() == 'Linux':
    sys.path.insert(0, os.path.expanduser('~/apps/arm-gnu-toolchain/share/gcc-14.2.1/python'))
else:
    print("Unsupported platform. Please add the libstdc++ printer path manually.")
try:
    from libstdcxx.v6.printers import register_libstdcxx_printers
    register_libstdcxx_printers(None)
    print("STL pretty printers loaded successfully.")
except ImportError:
    print("Error: Failed to load STL pretty printers.")
end
