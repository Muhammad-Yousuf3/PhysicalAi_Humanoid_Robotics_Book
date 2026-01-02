/**
 * Custom Layout component for Docusaurus.
 * Wraps the original Layout and adds ChatWidget within the React tree.
 * Uses BrowserOnly to prevent SSR hydration issues.
 */

import OriginalLayout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type { WrapperProps } from '@docusaurus/types';
import BrowserOnly from '@docusaurus/BrowserOnly';

type Props = WrapperProps<typeof LayoutType>;

export default function Layout(props: Props): JSX.Element {
  return (
    <OriginalLayout {...props}>
      {props.children}
      <BrowserOnly fallback={null}>
        {() => {
          const ChatWidget = require('../../components/ChatWidget').default;
          return <ChatWidget />;
        }}
      </BrowserOnly>
    </OriginalLayout>
  );
}
