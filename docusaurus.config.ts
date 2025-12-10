import rehypeKatex from "rehype-katex";
import remarkMath from "remark-math";

const config: Config = {
  title: 'PhysicalAI Humanoid Robotics',
  tagline: 'AI Systems for Embodied Intelligence',
  favicon: 'img/favicon.ico', // Will be replaced with a blank favicon

  // Set the production url of your site here
  url: 'https://muhammad-yousuf.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/PhysicalAi_Humanoid_Robotics_Book/',

  // GitHub pages deployment config.
  organizationName: 'muhammad-yousuf', // Usually your GitHub org/user name.
  projectName: 'PhysicalAi_Humanoid_Robotics_Book', // Usually your repo name.

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
            'https://github.com/muhammad-yousuf/PhysicalAi_Humanoid_Robotics_Book/tree/main/',
        },
        blog: false, // Removed blog
        theme: {
          customCss: './src/css/custom.css',
        },
      } 
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'PhysicalAI Humanoid Robotics', // Simplified title
      logo: {
        alt: 'PhysicalAI Humanoid Robotics Logo',
        src: 'img/logo.png', // Keeping a placeholder SVG, will modify later if needed.
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/muhammad-yousuf/PhysicalAi_Humanoid_Robotics_Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [], // Removed all links to simplify
      copyright: `Copyright © ${new Date().getFullYear()} Muhammad Yousuf. Built with Docusaurus.`,
    },

  } 
};

export default config;
