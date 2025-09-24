import os
import re
import shutil


def main():
    drivers_dir = os.path.join(".", "hal", "Drivers")
    if os.path.exists(drivers_dir):
        shutil.rmtree(drivers_dir)

    stm32f4xx_dir = os.path.join(
        "hal", "Drivers", "CMSIS", "Device", "ST", "STM32F4xx", "Include"
    )
    os.makedirs(stm32f4xx_dir, exist_ok=True)

    def copy_dir(src, dest):
        if not os.path.exists(src):
            print(f"警告: 源目录 {src} 不存在")
            return
        os.makedirs(dest, exist_ok=True)
        for item in os.listdir(src):
            s = os.path.join(src, item)
            d = os.path.join(dest, item)
            if os.path.isdir(s):
                shutil.copytree(s, d, dirs_exist_ok=True)
            else:
                shutil.copy2(s, d)

    copy_dir(os.path.join(".", "CMSIS_Device", "Include"), stm32f4xx_dir)

    copy_dir(
        os.path.join(".", "HAL_Driver"),
        os.path.join("hal", "Drivers", "STM32F4xx_HAL_Driver"),
    )

    copy_dir(
        os.path.join("..", "CMSIS5", "Core", "Include"),
        os.path.join("hal", "Drivers", "CMSIS", "Include"),
    )

    main_c_file = os.path.join("hal", "Core", "Src", "main.c")
    main_h_file = os.path.join("hal", "Core", "Inc", "main.h")

    if not os.path.exists(main_c_file):
        print(f"错误: 文件 {main_c_file} 不存在")
        return

    if not os.path.exists(main_h_file):
        print(f"错误: 文件 {main_h_file} 不存在")
        return

    with open(main_c_file, "r") as f:
        main_c_content = f.read()

    start_pattern = r"/\* Private variables [-]*\*/"
    end_pattern = r"/\* USER CODE BEGIN PV \*/"

    start_match = re.search(start_pattern, main_c_content)
    end_match = re.search(end_pattern, main_c_content)

    if not start_match or not end_match:
        print("未找到有效的变量声明区域")
        return

    start_idx = start_match.end()
    end_idx = end_match.start()
    variables_section = main_c_content[start_idx:end_idx]

    var_pattern = r"^\s*[A-Za-z_][A-Za-z0-9_]*\s+[A-Za-z_][A-Za-z0-9_]*\s*;"
    variables = re.findall(var_pattern, variables_section, re.MULTILINE)

    variables = [var.strip() for var in variables]

    if not variables:
        print("未找到有效的变量声明")
        return

    print("找到以下变量声明:")
    for var in variables:
        print(var)
    print()

    with open(main_h_file, "r") as f:
        main_h_content = f.readlines()

    et_line_index = None
    for i, line in enumerate(main_h_content):
        if "/* USER CODE BEGIN ET */" in line:
            et_line_index = i
            break

    if et_line_index is None:
        print("未找到 /* USER CODE BEGIN ET */ 标记")
        return

    for var in variables:
        var_decl = var.rstrip(";").strip()
        extern_decl = f"extern {var_decl};"

        exists = any(extern_decl.strip() in line.strip() for line in main_h_content)

        if not exists:
            main_h_content.insert(et_line_index + 1, f"  {extern_decl}\n")
            et_line_index += 1
            print(f"已添加: {extern_decl}")
        else:
            print(f"已存在: {extern_decl}")

    with open(main_h_file, "w") as f:
        f.writelines(main_h_content)

    print("处理完成")


if __name__ == "__main__":
    main()
