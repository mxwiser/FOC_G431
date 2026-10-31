1. 尽量只修改 /User /matlab /web 下的文件 ，如果修改 /User 不能完成的，
    与cubemx配置有关的，优先提示我修改CubeMx配置，其他无关的则询问我能不能修改。这样子是为了cubemx配置与工程代码同步，防止重新生成代码，导致我的代码被覆盖。
2. /matlab用于搭建仿真和创建临时测试文件，使用python。
3. AGENTS用的记忆文件，也可以放在/matlab下。
4. 这个工程是一个FOC项目工程。
5. 算法相关可以参考 https://github.com/vedderb/bldc
6. 嵌入式禁止使用c++动态特性。
7. /web 是web上位机。我会允npm run dev ,你只需要修改代码