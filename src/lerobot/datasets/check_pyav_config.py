import av

print("--- PyAV / FFmpeg Diagnostic Info ---")

# 打印PyAV自身的版本
print(f"PyAV version: {av.__version__}")
print("-" * 35)

# 打印FFmpeg各个库的版本
print("Linked FFmpeg library versions:")
for name, version_tuple in av.library_versions.items():
    version_str = '.'.join(map(str, version_tuple))
    print(f"  - {name}: {version_str}")
print("-" * 35)

# 打印最关键的信息：FFmpeg的编译配置
print("FFmpeg configuration flags:")
# av.configuration 是一个很长的字符串，我们把它格式化一下方便阅读
config_flags = av.configuration.split('--')
# 过滤掉空的字符串并重新加上 '--'
config_flags = [f"--{flag.strip()}" for flag in config_flags if flag.strip()]
for flag in sorted(config_flags): # 排序后更容易查找
    print(flag)

print("\n--- End of Report ---")