import shutil
import os
import uuid

files_to_copy = [
    (r"e:\puppyfangzhen\PuppyRobotPawn_patched.cpp", r"D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.cpp"),
    (r"e:\puppyfangzhen\PuppyRobotPawn_patched.h", r"D:\puppy_ue\Source\PuppyNav\PuppyRobotPawn.h"),
]

for src, dst in files_to_copy:
    print("处理: %s -> %s" % (src, dst))
    
    if not os.path.exists(src):
        print("  错误: 源文件不存在: %s" % src)
        continue
    
    success = False
    try:
        shutil.copy2(src, dst)
        print("  shutil.copy2 成功")
        success = True
    except Exception as e:
        print("  shutil.copy2 失败: %s" % str(e))
        print("  尝试 os.replace 与临时文件...")
        try:
            dst_dir = os.path.dirname(dst)
            tmp_name = os.path.join(dst_dir, "tmp_%s.tmp" % uuid.uuid4().hex)
            shutil.copy2(src, tmp_name)
            os.replace(tmp_name, dst)
            print("  os.replace 成功")
            success = True
        except Exception as e2:
            print("  os.replace 也失败: %s" % str(e2))
            if os.path.exists(tmp_name):
                try:
                    os.unlink(tmp_name)
                except:
                    pass
    
    if success and os.path.exists(dst):
        size = os.path.getsize(dst)
        print("  验证: 文件存在，大小: %d 字节" % size)
    else:
        print("  验证失败")
