import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

// https://astro.build/config
export default defineConfig({
  site: 'https://amarrmb.github.io',
  base: '/robotics-glossary',
  integrations: [
    starlight({
      title: 'Robotics Glossary',
      description: 'The canonical, community-maintained glossary for robotics concepts and the NVIDIA ecosystem',
      logo: {
        light: './src/assets/logo-light.svg',
        dark: './src/assets/logo-dark.svg',
        replacesTitle: false,
      },
      social: {
        github: 'https://github.com/amarrmb/robotics-glossary',
      },
      editLink: {
        baseUrl: 'https://github.com/amarrmb/robotics-glossary/edit/main/',
      },
      customCss: [
        './src/styles/custom.css',
      ],
      head: [
        {
          tag: 'meta',
          attrs: {
            property: 'og:image',
            content: '/robotics-glossary/og-image.png',
          },
        },
        {
          tag: 'meta',
          attrs: {
            name: 'twitter:card',
            content: 'summary_large_image',
          },
        },
      ],
      sidebar: [
        {
          label: 'Start Here',
          items: [
            { label: 'Introduction', slug: 'index' },
            { label: 'How to Use', slug: 'how-to-use' },
          ],
        },
        {
          label: 'Concepts',
          collapsed: false,
          items: [
            {
              label: 'Fundamentals',
              autogenerate: { directory: 'concepts/fundamentals' },
            },
            {
              label: 'Perception',
              autogenerate: { directory: 'concepts/perception' },
            },
            {
              label: 'Control & Planning',
              autogenerate: { directory: 'concepts/control' },
            },
            {
              label: 'AI & Learning',
              autogenerate: { directory: 'concepts/ai' },
            },
          ],
        },
        {
          label: 'Hardware',
          collapsed: false,
          items: [
            {
              label: 'Compute Platforms',
              items: [
                { label: 'Jetson Thor', slug: 'hardware/compute/jetson-thor' },
                { label: 'Jetson Orin', slug: 'hardware/compute/jetson-orin' },
                { label: 'DGX Spark', slug: 'hardware/compute/dgx-spark' },
              ],
            },
            {
              label: 'Sensors',
              autogenerate: { directory: 'hardware/sensors' },
            },
          ],
        },
        {
          label: 'Software',
          collapsed: false,
          items: [
            {
              label: 'Frameworks',
              items: [
                { label: 'ROS 2', slug: 'software/frameworks/ros2' },
              ],
            },
            {
              label: 'Isaac Platform',
              badge: { text: 'NVIDIA', variant: 'success' },
              items: [
                { label: 'Isaac ROS', slug: 'software/isaac/isaac-ros' },
                { label: 'Isaac Sim', slug: 'software/simulation/isaac-sim' },
              ],
            },
          ],
        },
        {
          label: 'Contributing',
          collapsed: true,
          items: [
            { label: 'How to Contribute', slug: 'contributing/how-to-contribute' },
            { label: 'Style Guide', slug: 'contributing/style-guide' },
          ],
        },
      ],
      components: {
        // Override components for custom styling
      },
      pagination: true,
      lastUpdated: true,
    }),
  ],
});
