---
title: '你好，世界：博客开张'
description: '博客的第一篇文章，记录这个小站的诞生过程。'
pubDate: '2026-10-26'
---

## 开张啦

这是本站的第一篇文章。这个小站用 [Astro](https://astro.build/) 搭建，托管在 GitHub Pages 上，完全免费。

## 为什么自己搭博客

- 写作的内容完全归自己所有，就是一堆 Markdown 文件
- `git push` 一下就能发布，顺便练习版本管理
- 没有广告、没有算法推荐、没有平台绑架

## 怎么写新文章

在 `src/content/blog/` 目录下新建一个 `.md` 文件，开头写好标题和日期：

```markdown
---
title: '文章标题'
description: '一句话摘要'
pubDate: '2026-10-26'
---

正文用 Markdown 随便写。
```

然后 `git push`，GitHub Actions 会自动构建并发布。

## 接下来

- [ ] 折腾评论系统
- [ ] 绑定自己的域名
- [ ] 坚持写下去

世界你好，我来了。
