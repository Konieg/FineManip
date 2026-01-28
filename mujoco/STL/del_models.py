import os
import shutil
from pathlib import Path

def safe_delete_files(folder_path, exclude_str, confirm=True):
    clean_path = folder_path.strip('''"' ''') 
    target_dir = Path(clean_path).expanduser().resolve() 
    
    if not target_dir.exists():
        raise ValueError(f"路径不存在：{target_dir}")
    
    if not target_dir.is_dir():
        raise ValueError(f"不是有效目录：{target_dir}")

    protected_dirs = [
        Path("/"), Path.home(),
        Path("C:\\Windows") if os.name == 'nt' else Path("/etc")
    ]
    if target_dir in protected_dirs:
        raise PermissionError("禁止操作系统目录")

    file_list = [f for f in target_dir.iterdir() if f.is_file()]

    exclude_clean = exclude_str.strip('''"''')
    to_delete = [
        f for f in file_list
        if exclude_clean.lower() not in f.name.lower()
    ]

    if not to_delete:
        print(f"没有需要删除的文件：{target_dir}")
        return 0

    print(f"发现 {len(to_delete)} 个待删除文件（示例）：")
    for i, f in enumerate(to_delete[:5], 1):
        print(f"[{i}] {f.name}")
    if len(to_delete) > 5:
        print(f"...等其他 {len(to_delete)-5} 个文件")

    if confirm:
        if input("确认删除？(y/n) ").lower() != 'y':
            print("操作取消")
            return 0

    success = 0
    for file in to_delete:
        try:
            if not file.exists():
                print(f"文件已消失：{file.name}")
                continue
                
            file.unlink()  # 删除文件
            print(f"已删除：{file.name}")
            success += 1
        except Exception as e:
            print(f"删除失败 [{file.name}]: {str(e)}")

    print(f"\n操作完成：成功删除 {success}/{len(to_delete)} 个文件")
    return success

if __name__ == "__main__":
    from pathlib import Path
    # 文件夹路径
    parent_path = Path(r"D:\safe RL\FinesRobot\fines\STL")
    predefined_paths = [
        "chassis",
        "base_link",
    ]
    path_list = [parent_path / folder_name for folder_name in predefined_paths]
    
    for path_str in path_list:
        exclude_str = path_str.name
        print(f"\n{'='*30}")
        print(f"处理路径：{path_str}")
        print(f"保留关键词：'{exclude_str}'")
        
        try:
            safe_delete_files(str(path_str), exclude_str, confirm=False)
        except Exception as e:
            print(f"错误：{str(e)}")
        print(f"{'='*30}\n")