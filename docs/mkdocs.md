# mkdocs简介

## readthedocs 主题


该主题是 OpenHUTB 文档默认主题，即该页面所看到的效果。

要选择主题，请在 mkdocs.yml 配置文件中设置主题配置选项。

```yml
theme:
  name: readthedocs
```

该主题是 OpenHUTB 文档默认主题，即该页面所看到的效果。
这也是 mkdocs 仓库的衍生版，提供的功能集与其父主题相同，且均受到限制。与父主题一样，它也仅支持两级导航。


除了默认的主题配置选项外，`readthedocs` 主题还支持以下选项：

* **`highlightjs`**：启用使用 [highlight.js](https://highlightjs.org/) JavaScript 库对代码块中的源代码进行高亮显示。默认值：`True`。

* **`hljs_languages`**：highlight.js 默认仅支持 23 种常用语言。在此处列出其他语言以添加支持。

    ```yml
    theme:
      name: readthedocs
      highlightjs: true
      hljs_languages:
          - yaml
          - rust
    ```

* **`analytics`**：定义分析服务的配置选项。

    * `gtag`：要启用 Google Analytics（分析），请将其设置为 Google Analytics（分析）v4 跟踪 ID，该 ID 使用 `G-` 格式。请参阅 Google 文档，了解[如何为网站和/或应用设置 Analytics（分析）（GA4）](https://support.google.com/analytics/answer/9304153?hl=en&ref_topic=9303319)或[升级到 Google Analytics（分析）4 媒体资源](https://support.google.com/analytics/answer/9744165?hl=en&ref_topic=9303319)。
    ```yml
    theme:
      name: readthedocs
      analytics:
        gtag: G-ABC123
    ```
    当设置为默认值（null）时，Google Analytics（分析）功能将被禁用。

    * `anonymize_ip`：要为 Google Analytics（分析）启用匿名 IP 地址，请将其设置为 True。默认值：`False`。

*   **`include_homepage_in_sidebar`**: 在侧边栏菜单中显示首页。由于 MkDocs 要求在导航（`nav`）配置选项中列出首页，因此此设置允许将首页包含在侧边栏中或从侧边栏中排除。请注意，网站名称/徽标始终链接到首页。默认值：`True`。

*   **`prev_next_buttons_location`**: 选项包括：底部（`bottom`）、顶部（`top`）、顶部和底部（ `both`）、或都不显示（`none`）。根据选项显示“下一页”和“上一页”按钮。默认值：底部（`bottom`）。 

*   **`navigation_depth`**: 侧边栏导航树的最大深度。默认值：`4`。

*   **`collapse_navigation`**: 仅在当前页面的侧边栏中包含页面章节标题。默认值：`True`。

*   **`titles_only`**: 侧边栏仅包含页面标题，不包含所有页面的所有章节标题。默认值：`False`（最好有右边的页面内导航才启用）。

*   **`sticky_navigation`**: 如果为 True，则侧边栏会随着页面滚动而滚动。默认值：`True`。

*   **`locale`**：用于构建主题的语言/地区设置。如果您的语言/地区尚不受支持，则会回退到默认设置。

    此主题支持以下语言/地区：

    * `en`: 英语（默认）
    * (查看现有目录列表 `mkdocs/themes/readthedocs/locales/*/`)

    有关更多信息，请参阅[主题本地化]指南。

*   **`logo`**: 要在项目中使用徽标而不是纯文本 `site_name`，请将此变量设置为图像的位置。默认值：`null`。 



