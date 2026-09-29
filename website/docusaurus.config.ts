import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';
import repoLinks from './src/remark/repo-links.mjs';

const repo = 'https://github.com/fl4p/fugu-mppt-firmware';


const config: Config = {
  title: 'Fugu MPPT Firmware',
  tagline: 'Firmware for ESP32 MPPT solar chargers and DC/DC converters',
  favicon: 'img/favicon.svg',
  url: 'https://fl4p.github.io',
  baseUrl: '/fugu-mppt-firmware/',
  organizationName: 'fl4p',
  projectName: 'fugu-mppt-firmware',
  trailingSlash: true,
  onBrokenLinks: 'throw',
  onBrokenAnchors: 'throw',
  markdown: {
    format: 'detect',
    mermaid: true,
    hooks: {onBrokenMarkdownLinks: 'throw'},
  },
  themes: [
    '@docusaurus/theme-mermaid',
    [
      '@easyops-cn/docusaurus-search-local',
      {hashed: true, indexBlog: false},
    ],
  ],
  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.16.47/dist/katex.min.css',
      type: 'text/css',
      crossorigin: 'anonymous',
    },
  ],
  i18n: {defaultLocale: 'en', locales: ['en']},
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: `${repo}/edit/main/website/`,
          beforeDefaultRemarkPlugins: [
            [repoLinks, {repoUrl: repo}],
          ],
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
          showLastUpdateTime: true,
        },
        blog: false,
        theme: {customCss: './src/css/custom.css'},
      } satisfies Preset.Options,
    ],
  ],
  themeConfig: {
    colorMode: {respectPrefersColorScheme: true},
    navbar: {
      title: 'Fugu MPPT',
      logo: {alt: 'Fugu', src: 'img/favicon.svg'},
      items: [
        {type: 'docSidebar', sidebarId: 'guide', position: 'left', label: 'Guide'},
        {type: 'docSidebar', sidebarId: 'reference', position: 'left', label: 'Reference'},
        {type: 'docSidebar', sidebarId: 'internals', position: 'left', label: 'How it works'},
        {type: 'docSidebar', sidebarId: 'lab', position: 'left', label: 'Lab'},
        {type: 'docSidebar', sidebarId: 'development', position: 'left', label: 'Development'},
        {href: repo, label: 'GitHub', position: 'right'},
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Project',
          items: [
            {label: 'Firmware', href: repo},
            {label: 'Fugu2 hardware', href: 'https://github.com/fl4p/Fugu2'},
            {label: 'fugu-py console client', href: 'https://github.com/fl4p/fugu-py'},
          ],
        },
        {
          title: 'Docs',
          items: [
            {label: 'Guide', to: '/docs/guide/intro'},
            {label: 'Reference', to: '/docs/reference/config'},
            {label: 'How it works', to: '/docs/internals/architecture'},
          ],
        },
      ],
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'ini', 'cpp', 'python', 'powershell', 'cmake'],
    },
    tableOfContents: {minHeadingLevel: 2, maxHeadingLevel: 4},
  } satisfies Preset.ThemeConfig,
};

export default config;
