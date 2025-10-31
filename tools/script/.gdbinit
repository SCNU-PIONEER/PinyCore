# .gdbinit
set p obj on
set p pretty on

python
import os
import sys

current_dir = os.getcwd()
toolchain_path = os.path.join(current_dir,'tools','script','python')
print(f"Using path: {toolchain_path}")

if os.path.exists(toolchain_path):
    sys.path.insert(0, toolchain_path)
    try:
        from libstdcxx.v6.printers import register_libstdcxx_printers
        register_libstdcxx_printers(None)
        print("Successfully loaded libstdcxx printers")
    except ImportError as e:
        print(f"Failed to load libstdcxx printers: {e}")
else:
    print(f"Toolchain path does not exist: {toolchain_path}")
end
