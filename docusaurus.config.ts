import type { Config } from '@docusaurus/types';
import rehypeKatex from "rehype-katex";
import remarkMath from "remark-math";

const config: Config = {
  title: 'PhysicalAI Humanoid Robotics',
  tagline: 'AI Systems for Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Production URL of your site on Vercel
  url: 'https://physical-ai-humanoid-robotics-book-navy.vercel.app',
  baseUrl: '/',

  // Remove runtime-config script; we will use env via customFields
  // scripts: [{ src: "/runtime-config.js", async: false }], // Removed

  // Custom fields for frontend access (API URL)
  customFields: {
    apiUrl: process.env.REACT_APP_API_URL || 'https://muhammadyousuf333-rag-chatbot-with-book.hf.space/api',
  },

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.13.24/dist/katex.min.css',
      type: 'text/css',
      integrity:
        'sha384-odtC+0UGzzFL/6PNoE8rX/SPCSSL65cRO+j6HETArO/yemkLZltD825m+Cj0M7l6',
      crossorigin: 'anonymous',
    },
    // Urdu font for translations
    {
      href: 'https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap',
      type: 'text/css',
    },
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
          editUrl:
            'https://github.com/Muhammad3-Yousuf/PhysicalAi_Humanoid_Robotics_Book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'PhysicalAI Humanoid Robotics',
      logo: {
        alt: 'PhysicalAI Humanoid Robotics Logo',
        src: 'img/logo.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/Muhammad-Yousuf3/PhysicalAi_Humanoid_Robotics_Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [],
      copyright: `Copyright © ${new Date().getFullYear()} Muhammad Yousuf. Built with Docusaurus.`,
    },
  },
};

export default config;
