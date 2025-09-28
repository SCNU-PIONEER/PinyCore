import re
import os
import argparse
import json

def load_structures_from_json(json_file_path):
    """从JSON文件加载结构体数据"""
    try:
        with open(json_file_path, 'r', encoding='utf-8') as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"错误: JSON文件不存在: {json_file_path}")
        return []
    except json.JSONDecodeError:
        print(f"错误: JSON文件格式错误: {json_file_path}")
        return []

def parse_struct_definition(struct_definition):
    """解析结构体定义，返回字段列表，处理多个同名的reserved字段"""
    fields = []
    reserved_count = 0 
    
    for field in struct_definition.split(';'):
        field = field.strip()
        if field:
            if ':' in field:
                parts = field.split(':')
                type_and_name = parts[0].strip()
                bit_width = parts[1].strip()
                if ' ' in type_and_name:
                    field_type, field_name = type_and_name.rsplit(' ', 1)
                    if field_name == 'reserved':
                        reserved_count += 1
                        field_name = f"reserved{reserved_count}"
                    fields.append(f"{field_type} {field_name} : {bit_width}")
                else:
                    fields.append(field)
            else:
                if ' ' in field:
                    field_type, field_name = field.rsplit(' ', 1)
                    if field_name == 'reserved':
                        reserved_count += 1
                        field_name = f"reserved{reserved_count}"
                    fields.append(f"{field_type} {field_name}")
                else:
                    fields.append(field)
    
    return fields

def extract_existing_struct_fields(content, struct_name):
    """从内容中提取现有结构体的字段定义"""
    exact_pattern = r'struct\s+' + re.escape(struct_name) + r'\s*\{([^}]*)\}'
    match = re.search(exact_pattern, content, re.DOTALL)
    if match:
        fields_content = match.group(1)
        fields = []
        for line in fields_content.split('\n'):
            line = line.strip()
            if line and not line.startswith('}'):
                if line.endswith(';'):
                    line = line[:-1]
                fields.append(line)
        print(f"使用精确匹配找到结构体 {struct_name}，字段: {fields}")
        return fields
    
    print(f"未找到结构体 {struct_name}")
    return None

def find_info_num_position(content):
    """查找 INFO_NUM 常量的位置"""
    info_num_pattern = r'uint8_t\s+constexpr\s+INFO_NUM\s*=\s*\d+\s*;'
    match = re.search(info_num_pattern, content)
    
    if match:
        return match.start()

    return -1

def find_info_array_position(content):
    """查找 INFO 数组的位置"""
    info_array_pattern = r'INFO_s\s+constexpr\s+INFO\s*\[\s*INFO_NUM\s*\]\s*=\s*\{'
    match = re.search(info_array_pattern, content)
    
    if match:
        return match.start()

    return -1

def extract_info_array_entries(content):
    """从内容中提取 INFO 数组的条目"""
    info_array_start_pattern = r'INFO_s\s+constexpr\s+INFO\s*\[\s*INFO_NUM\s*\]\s*=\s*\{'
    info_array_end_pattern = r'\};'
    
    start_match = re.search(info_array_start_pattern, content)
    if not start_match:
        return None

    brace_count = 1
    array_start_pos = start_match.end()
    array_end_pos = array_start_pos
    
    for i in range(array_start_pos, len(content)):
        if content[i] == '{':
            brace_count += 1
        elif content[i] == '}':
            brace_count -= 1
            if brace_count == 0:
                array_end_pos = i
                break

    array_content = content[array_start_pos:array_end_pos]

    entries = []
    current_entry = ""
    brace_count = 0
    
    for char in array_content:
        current_entry += char
        if char == '{':
            brace_count += 1
        elif char == '}':
            brace_count -= 1
            if brace_count == 0:
                entry = current_entry.strip()
                if entry:
                    entries.append(entry)
                current_entry = ""
    
    return entries

def extract_existing_enum_ids(content):
    """从内容中提取现有的枚举ID"""
    enum_pattern = r'enum class CmdId_e\s*:\s*\w+\s*\{(.*?)\};'
    match = re.search(enum_pattern, content, re.DOTALL)
    
    if match:
        enum_content = match.group(1)
        enum_ids = []
        for line in enum_content.split('\n'):
            line = line.strip()
            if '=' in line:
                enum_name = line.split('=')[0].strip()
                if enum_name:
                    enum_ids.append(enum_name)
        return enum_ids
    
    return []

def replace_struct_definition(content, struct_name, new_struct_def):
    """替换结构体定义，确保只保留一个空行"""
    patterns = [
        r'(struct\s+' + re.escape(struct_name) + r'\s*\{.*?\}\s*;\s*\n)(\s*\n)*',
        r'(struct\s+' + re.escape(struct_name) + r'\s*\{.*?\}\s*)(\s*\n)*',
        r'(struct\s+' + re.escape(struct_name) + r'\s*\{.*?\}\s*;\s*)(\s*\n)*',
        r'(struct\s+' + re.escape(struct_name) + r'\s*\{.*?\}\s*)(\s*\n)*'
    ]
    
    original_content = content
    for i, pattern in enumerate(patterns):
        print(f"尝试使用模式 {i+1} 替换结构体 {struct_name}")
        def replace_func(match):
            return new_struct_def + '\n'
        
        new_content = re.sub(pattern, replace_func, content, flags=re.DOTALL)
        if new_content != content:
            print(f"使用模式 {i+1} 成功替换结构体 {struct_name}")
            return new_content
    
    print(f"所有替换方法都失败")
    return original_content

def add_field_to_referee_prot(content, struct_name):
    """将新结构体添加到 RefereeProt_s 结构体中，字段名第一个字母小写，检查是否已存在相同字段名"""
    referee_prot_fields = extract_existing_struct_fields(content, "RefereeProt_s")
    
    if referee_prot_fields is None:
        print("警告: 未找到 RefereeProt_s 结构体")
        return content
    
    base_name = struct_name[:-2]
    field_name = base_name[0].lower() + base_name[1:]
    
    field_name_lower = field_name.lower()
    for field in referee_prot_fields:
        if ' ' in field:
            existing_field_name = field.split(' ')[1].lower()
            if existing_field_name == field_name_lower:
                print(f"字段名 {field_name}（不区分大小写）已存在于 RefereeProt_s 中，跳过添加")
                return content
    
    field_def = f"{struct_name} {field_name}"
    
    referee_prot_fields.append(field_def)
    
    new_referee_prot_def = "struct RefereeProt_s {\n"
    for field in referee_prot_fields:
        new_referee_prot_def += f"    {field};\n"
    new_referee_prot_def += "};"
    
    return replace_struct_definition(content, "RefereeProt_s", new_referee_prot_def)

def add_entry_to_info_array(content, struct_name, enum_name, field_name):
    """将新结构体添加到 INFO 数组中"""
    info_array_pos = find_info_array_position(content)
    
    if info_array_pos == -1:
        print("警告: 未找到 INFO 数组")
        return content
    
    existing_entries = extract_info_array_entries(content)
    
    if existing_entries is None:
        print("警告: 无法提取 INFO 数组条目")
        return content
    
    for entry in existing_entries:
        if f'.cmdId = CmdId_e::{enum_name}' in entry:
            print(f"cmdId {enum_name} 已存在于 INFO 数组中，跳过添加")
            return content
    
    new_entry = f"    {{ .cmdId = CmdId_e::{enum_name},\n      .offsetByte = offsetof(RefereeProt_s, {field_name}),\n      .size = sizeof({struct_name}) }}"
    
    info_array_end_pattern = r'\};'
    end_match = re.search(info_array_end_pattern, content[info_array_pos:])
    
    if not end_match:
        print("警告: 无法找到 INFO 数组的结束位置")
        return content
    
    end_pos = info_array_pos + end_match.start()
    
    last_entry_pos = -1
    for i in range(end_pos - 1, info_array_pos, -1):
        if content[i] == '}':
            last_entry_pos = i
            break
    
    if last_entry_pos == -1:
        print("警告: 无法找到 INFO 数组的最后一个条目")
        return content
    
    new_content = content[:last_entry_pos + 1] + ",\n" + new_entry + content[last_entry_pos + 1:]
    
    return new_content

def ensure_struct_name_suffix(struct_name):
    """确保结构体名称有 _s 后缀"""
    if not struct_name.endswith('_s'):
        return struct_name + '_s'
    return struct_name

def modify_source_file(structures_data):
    """
    修改源文件，添加多个结构体定义和枚举值
    
    参数:
        structures_data: 结构体数据列表，每个元素是一个字典，包含:
            - struct_definition: 结构体定义字符串
            - new_struct_name: 新结构体名称
            - cmd_id: 命令ID
            - enum_name: 枚举名称
    """
    try:
        source_file_path = os.path.join(os.path.dirname(__file__), "../../Src/Common/Extensions/Referee/RefereeProt.hpp")
        print(f"正在处理文件: {source_file_path}")
        
        if not os.path.exists(source_file_path):
            raise FileNotFoundError(f"文件不存在: {source_file_path}")

        with open(source_file_path, 'r', encoding='utf-8') as file:
            content = file.read()
        
        print(f"文件大小: {len(content)} 字符")

        existing_enum_ids = extract_existing_enum_ids(content)
        
        modified_content = content
        added_enum_ids = set()
        
        for i, struct_data in enumerate(structures_data):
            if i > 0:
                print()
                
            struct_definition = struct_data['struct_definition']
            new_struct_name = struct_data['new_struct_name']
            cmd_id = struct_data['cmd_id']
            enum_name = struct_data['enum_name']
            
            new_struct_name = ensure_struct_name_suffix(new_struct_name)

            new_fields = parse_struct_definition(struct_definition)
            print(f"新结构体 {new_struct_name} 的字段: {new_fields}")
            
            existing_fields = extract_existing_struct_fields(content, new_struct_name)
            
            if existing_fields is not None:
                print(f"结构体 {new_struct_name} 已存在，比较字段")
                print(f"现有字段: {existing_fields}")
                print(f"新字段: {new_fields}")
                
                if set(existing_fields) == set(new_fields):
                    print(f"结构体 {new_struct_name} 已存在且字段一致，跳过")
                    continue
                else:
                    print(f"结构体 {new_struct_name} 已存在但字段不一致，将更新")

                    new_struct_def = f"struct {new_struct_name} {{\n"
                    for field in new_fields:
                        new_struct_def += f"    {field};\n"
                    new_struct_def += "};" 

                    temp_content = replace_struct_definition(modified_content, new_struct_name, new_struct_def)

                    if temp_content == modified_content:
                        print(f"警告: 结构体 {new_struct_name} 替换可能失败")
                    else:
                        print(f"成功更新结构体 {new_struct_name}")
                        modified_content = temp_content
            else:
                print(f"添加新结构体 {new_struct_name}")
                
                new_struct_def = f"struct {new_struct_name} {{\n"
                for field in new_fields:
                    new_struct_def += f"    {field};\n"
                new_struct_def += "};"  

                info_num_pos = find_info_num_position(modified_content)
                
                if info_num_pos != -1:
                    lines = modified_content.split('\n')
                    info_num_line = -1
                    char_count = 0
                    for i, line in enumerate(lines):
                        char_count += len(line) + 1
                        if char_count > info_num_pos:
                            info_num_line = i
                            break
                    
                    if info_num_line != -1:
                        new_lines = lines[:info_num_line] + ['', new_struct_def, '']
                        if info_num_line < len(lines):
                            new_lines.extend(lines[info_num_line:])
                        modified_content = '\n'.join(new_lines)
                        print(f"在 INFO_NUM 常量前添加新结构体 {new_struct_name}")

                        modified_content = add_field_to_referee_prot(modified_content, new_struct_name)
                        
                        base_name = new_struct_name[:-2]
                        field_name = base_name[0].lower() + base_name[1:]

        content = modified_content

        enum_pattern = r'(enum class CmdId_e\s*:\s*\w+\s*\{)(.*?)(\s*};)'
        match = re.search(enum_pattern, content, re.DOTALL)
        
        if match:
            enum_content = match.group(2)
            lines = enum_content.split('\n')
            insert_pos = -1
            for i in range(len(lines)-1, -1, -1):
                if '=' in lines[i]:
                    if re.match(r'\s*\w+\s*=\s*0x[0-9A-Fa-f]+', lines[i]):
                        insert_pos = i
                        break
            
            if insert_pos != -1:
                new_enum_entries = []
                for struct_data in structures_data:
                    cmd_id = struct_data['cmd_id']
                    enum_name = struct_data['enum_name']

                    id_pattern = r'=\s*0x' + format(cmd_id, '04X')
                    if re.search(id_pattern, content):
                        print(f"警告: 枚举值 ID {hex(cmd_id)} 已存在，跳过添加")
                        continue
                        
                    new_enum_entries.append((enum_name, f'    {enum_name} = {hex(cmd_id)},'))
                
                if new_enum_entries:
                    enum_names, enum_entries = zip(*new_enum_entries)
                    new_lines = lines[:insert_pos+1] + list(enum_entries) + lines[insert_pos+1:]
                    new_enum_content = '\n'.join(new_lines)
                    new_enum = match.group(1) + new_enum_content + match.group(3)
                    content = content.replace(match.group(0), new_enum)
                    print(f"在枚举中添加了 {len(enum_entries)} 个新的ID")

                    added_enum_ids.update(enum_names)
            else:
                print("警告: 未找到有效的枚举位置")

        if added_enum_ids:
            print()
            
        for i, struct_data in enumerate(structures_data):
            enum_name = struct_data['enum_name']
            new_struct_name = struct_data['new_struct_name']
            
            new_struct_name = ensure_struct_name_suffix(new_struct_name)
            
            if enum_name in added_enum_ids:
                if i > 0 and enum_name in added_enum_ids:
                    pass 
                    
                print(f"为枚举ID {enum_name} 添加对应的INFO数组条目")

                if new_struct_name.endswith('_s'):
                    base_name = new_struct_name[:-2]
                    field_name = base_name[0].lower() + base_name[1:]
                else:
                    field_name = new_struct_name[0].lower() + new_struct_name[1:]

                content = add_entry_to_info_array(content, new_struct_name, enum_name, field_name)

        with open(source_file_path, 'w', encoding='utf-8') as file:
            file.write(content)

        print(f"文件 {source_file_path} 已成功修改")

    except FileNotFoundError as e:
        print(f"错误: {e}")
    except Exception as e:
        print(f"处理文件时发生错误: {e}")
        import traceback
        traceback.print_exc()

def main():
    parser = argparse.ArgumentParser(description='修改源文件，添加多个结构体定义和枚举值')
    parser.add_argument('--json-file',help='tools/script/structs.json')
    
    args = parser.parse_args()
    
    structures_data = load_structures_from_json(args.json_file)
    
    if not structures_data:
        print("错误: 无法加载结构体数据")
        return
        
    modify_source_file(structures_data)

if __name__ == "__main__":
    main()
