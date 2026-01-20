# Git仓库使用指南

## 仓库信息

**仓库位置**：`D:\Project\MCU\RobotChassis_Firmware`
**分支**：main（默认分支）
**远程仓库**：未配置（可选择性添加）

---

## 当前状态

✅ Git仓库已初始化
✅ .gitignore已配置（忽略build/、.vscode/等）
✅ 首次提交已完成
✅ README.md文档已添加

**提交历史**：
```
35609b6 - docs: add comprehensive README.md
2ebac15 - Initial commit: STM32 RobotChassis Firmware
```

---

## 日常开发工作流

### 1. 查看状态

```bash
cd "D:\Project\MCU\RobotChassis_Firmware"
git status
```

### 2. 添加修改的文件

```bash
# 添加所有修改
git add .

# 添加特定文件
git add USER/MotorControl/motor_control.c

# 查看暂存的文件
git status
```

### 3. 提交更改

```bash
# 提交并添加说明
git commit -m "feat(MotorControl): 优化PID控制算法"

# 添加详细说明
git commit -m "fix(IMU): 修复DMP初始化失败问题

- 检查I2C连接
- 增加重试机制
- 添加错误日志"
```

### 4. 查看历史

```bash
# 查看最近5次提交
git log --oneline -5

# 查看详细信息
git log -2 --stat

# 图形化显示
git log --graph --oneline --all
```

### 5. 撤销修改

```bash
# 撤销工作区修改（恢复到上次提交）
git checkout -- <file>

# 撤销暂存区的修改
git reset HEAD <file>

# 撤销最后一次提交（保留修改）
git reset --soft HEAD~1

# 撤销最后一次提交（丢弃修改）
git reset --hard HEAD~1  # 危险操作！
```

---

## Commit规范

### Commit消息格式

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Type类型

| Type | 说明 | 示例 |
|------|------|------|
| `feat` | 新功能 | `feat(MotorControl): 添加速度平滑控制` |
| `fix` | 修复bug | `fix(IMU): 修复DMP初始化失败` |
| `docs` | 文档 | `docs: 更新README说明` |
| `style` | 格式 | `style: 统一代码缩进` |
| `refactor` | 重构 | `refactor(Comm): 优化协议解析逻辑` |
| `test` | 测试 | `test: 添加电机单元测试` |
| `chore` | 构建/工具 | `chore: 更新CMake配置` |
| `perf` | 性能优化 | `perf(IMU): 优化数据读取速度` |

### Scope范围

- `MotorControl` - 电机控制
- `IMU` - 惯性测量单元
- `Communication` - 通信协议
- `ServoControl` - 舵机控制
- `PowerManagement` - 电源管理
- `LedControl` - LED控制
- `System` - 系统级代码
- `Core` - CubeMX生成的代码

### 示例

```
feat(MotorControl): 实现速度闭环PID控制

- 添加增量式PID算法
- 支持动态参数调整
- 优化响应速度

Closes #123
```

---

## .gitignore说明

已配置忽略以下文件/目录：

### 构建输出
- `build/` - CMake构建目录
- `Debug/`, `Release/` - 构建输出

### 编译产物
- `*.o`, `*.obj` - 目标文件
- `*.elf`, `*.bin`, `*.hex` - 二进制文件
- `*.map` - 映射文件

### CMake生成
- `CMakeCache.txt` - CMake缓存
- `CMakeFiles/` - CMake生成目录

### IDE配置
- `.vscode/` - VSCode配置
- `.idea/` - CLion配置
- `*.launch` - Eclipse调试配置

### 其他
- `*.log` - 日志文件
- `*.tmp`, `*.bak` - 临时文件
- `.DS_Store` - macOS系统文件

**注意**：如果需要追踪某些被忽略的文件，使用：
```bash
git add -f <file>
```

---

## 分支管理

### 查看分支
```bash
git branch -a
```

### 创建新分支
```bash
# 创建并切换到新分支
git checkout -b feature/new-feature

# 或者分两步
git branch feature/new-feature
git checkout feature/new-feature
```

### 合并分支
```bash
# 切换到main分支
git checkout main

# 合并feature分支
git merge feature/new-feature

# 删除已合并的分支
git branch -d feature/new-feature
```

---

## 标签管理

### 创建标签
```bash
# 轻量级标签
git tag v1.0.0

# 带注释的标签
git tag -a v1.0.0 -m "Release version 1.0.0"

# 查看标签
git tag

# 查看标签详情
git show v1.0.0
```

### 推送标签到远程
```bash
git push origin v1.0.0
```

---

## 添加远程仓库

### GitHub

```bash
# 添加远程仓库
git remote add origin https://github.com/username/RobotChassis_Firmware.git

# 推送到远程
git push -u origin main

# 或使用SSH（推荐）
git remote add origin git@github.com:username/RobotChassis_Firmware.git
git push -u origin main
```

### Gitee（码云）

```bash
git remote add origin https://gitee.com/username/RobotChassis_Firmware.git
git push -u origin main
```

---

## 常用命令速查

| 操作 | 命令 |
|------|------|
| 查看状态 | `git status` |
| 查看日志 | `git log --oneline -10` |
| 查看分支 | `git branch` |
| 切换分支 | `git checkout <branch>` |
| 添加文件 | `git add <file>` |
| 提交 | `git commit -m "message"` |
| 推送 | `git push` |
| 拉取 | `git pull` |
| 查看差异 | `git diff` |
| 查看暂存差异 | `git diff --staged` |
| 撤销工作区修改 | `git checkout -- <file>` |
| 撤销暂存 | `git reset HEAD <file>` |

---

## 最佳实践

### 1. 频繁提交
- 小步快跑，每次提交一个功能点
- 不要积累太多修改再提交

### 2. 写好Commit消息
- 使用规范的格式
- 清晰描述做了什么
- 必要时说明为什么这样做

### 3. 使用分支
- 新功能在新分支开发
- 测试通过后再合并到main

### 4. 定期推送
- 工作完成后推送到远程仓库
- 避免代码丢失

### 5. 代码审查
- 重要修改提交PR
- 让他人review代码

---

## 项目特定建议

### STM32项目开发流程

```bash
# 1. 创建功能分支
git checkout -b feature/new-pid-algorithm

# 2. 修改代码
# ... 编辑代码 ...

# 3. 测试编译
cmake --preset Debug
cmake --build build/Debug

# 4. 提交修改
git add USER/MotorControl/
git commit -m "feat(MotorControl): 添加新的PID算法"

# 5. 合并到main
git checkout main
git merge feature/new-pid-algorithm

# 6. 删除功能分支
git branch -d feature/new-pid-algorithm
```

### CubeMX修改流程

```bash
# 1. 使用CubeMX修改.ioc文件
# 2. 重新生成代码
# 3. 检查Git状态
git status

# 4. 查看Core/目录的变化
git diff Core/Src/main.c

# 5. 提交修改
git add Core/
git commit -m "chore(Core): 更新CubeMX配置 - 添加新UART"
```

---

## 故障排除

### 问题1：文件未被追踪
```
Untracked files: build/
```
**解决**：这些文件已被.gitignore忽略，无需提交

### 问题2：提交后发现写错了
```bash
# 修改最后一次提交
git commit --amend -m "正确的提交消息"
```

### 问题3：想查看某个文件的修改历史
```bash
git log --follow -- USER/MotorControl/motor_control.c
```

### 问题4：想恢复某个文件到指定版本
```bash
git checkout <commit-hash> -- <file>
```

---

## 下一步

### 推荐操作

1. **添加远程仓库**（如果需要）
   ```bash
   git remote add origin <repository-url>
   git push -u origin main
   ```

2. **创建开发分支**
   ```bash
   git checkout -b dev
   ```

3. **设置Git别名**（可选）
   ```bash
   git config --global alias.st status
   git config --global alias.co checkout
   git config --global alias.br branch
   git config --global alias.ci commit
   ```

4. **配置全局.gitignore**（可选）
   在用户目录创建 `~/.gitignore_global`
   ```bash
   # Windows
   Thumbs.db
   Desktop.ini

   # macOS
   .DS_Store
   ```

---

## 参考资料

- [Git官方文档](https://git-scm.com/doc)
- [Pro Git 中文版](https://git-scm.com/book/zh/v2)
- [GitHub Flow](https://docs.github.com/en/get-started/quickstart/github-flow)

---

**Happy Coding! 🚀**
