# branch
1. **主分支(master)**
   - 项目的唯一长期分支
   - 包含所有将被包含在下个发布版本的代码
   - 只有通过代码审查的变更才能合并入内

2. **开发分支（develop)**
   ```bash
   # 标准操作流程示例
   git checkout main
   git pull
   git checkout -b feature/new-auth-system  # 从main创建功能分支
   # 开发完成后...
   git push origin feature/new-auth-system
   # 创建Pull Request等待审核
   ```

3. **发布流程**
   - 当master分支达到稳定状态时：
     - 创建版本提交(version bump commit)
     - 打上版本标签(如v1.3.0)
     - 生成发布产物(二进制包等)

4. **热修复流程**
   ```bash
   # 热修复示例(针对v1.2.0版本的修复)
   git checkout -b hotfix/login-bug v1.2.0  # 从标签创建
   # 修复问题后...
   git tag -a v1.2.1 -m "紧急修复登录漏洞"
   git push origin v1.2.1
   ```

5. **特性分支(feature)工作流增强**
   - 建议命名规范：`feature/<issue-id>-<short-desc>`
   - 推荐在PR中关联issue：`Fix #123 - Add user auth`

6. **发布流程检查清单**：
   - [ ] 所有CI测试通过
   - [ ] 版本号已更新
   - [ ] CHANGELOG.md已更新
   - [ ] 所有依赖项已验证
