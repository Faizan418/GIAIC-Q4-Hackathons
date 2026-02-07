import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '../components/chat/ChatWidget';

type LayoutProps = {
  children: React.ReactNode;
};

export default function Layout(props: LayoutProps): React.ReactElement {
  return (
    <OriginalLayout {...props}>
      {props.children}
      <ChatWidget />
    </OriginalLayout>
  );
}
