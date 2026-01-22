import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

const POSTHOG_KEY = process.env.PUBLIC_POSTHOG_KEY || '';
const POSTHOG_HOST = process.env.PUBLIC_POSTHOG_HOST || 'https://us.i.posthog.com';

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
        // PostHog Analytics - only loads if key is configured
        ...(POSTHOG_KEY ? [{
          tag: 'script',
          content: `
            !function(t,e){var o,n,p,r;e.__SV||(window.posthog=e,e._i=[],e.init=function(i,s,a){function g(t,e){var o=e.split(".");2==o.length&&(t=t[o[0]],e=o[1]),t[e]=function(){t.push([e].concat(Array.prototype.slice.call(arguments,0)))}}(p=t.createElement("script")).type="text/javascript",p.async=!0,p.src=s.api_host+"/static/array.js",(r=t.getElementsByTagName("script")[0]).parentNode.insertBefore(p,r);var u=e;for(void 0!==a?u=e[a]=[]:a="posthog",u.people=u.people||[],u.toString=function(t){var e="posthog";return"posthog"!==a&&(e+="."+a),t||(e+=" (stub)"),e},u.people.toString=function(){return u.toString(1)+".people (stub)"},o="capture identify alias people.set people.set_once set_config register register_once unregister opt_out_capturing has_opted_out_capturing opt_in_capturing reset isFeatureEnabled getFeatureFlag getFeatureFlagPayload reloadFeatureFlags group updateEarlyAccessFeatureEnrollment getEarlyAccessFeatures getActiveMatchingSurveys getSurveys onFeatureFlags onSessionId".split(" "),n=0;n<o.length;n++)g(u,o[n]);e._i.push([i,s,a])},e.__SV=1)}(document,window.posthog||[]);
            posthog.init('${POSTHOG_KEY}', {
              api_host: '${POSTHOG_HOST}',
              capture_pageview: true,
              capture_pageleave: true,
            });
          `,
        }] : []),
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
