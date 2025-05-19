# .gdbinit
python
import sys
# 添加libstdc++打印脚本路径（根据实际路径修改）
sys.path.insert(0, '/home/yjy/apps/arm-gnu-toochain/share/gcc-13.3.1/python')
try:
    from libstdcxx.v6.printers import register_libstdcxx_printers
    register_libstdcxx_printers(None)
    print("STL pretty printers loaded successfully.")
except ImportError:
    print("Error: Failed to load STL pretty printers.")
end