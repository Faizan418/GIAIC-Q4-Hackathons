import React from 'react';
import Layout from '@theme/Layout';
import { ProtectedRoute } from '../services/auth/protected-route';

interface ProtectedDocsLayoutProps {
  children: React.ReactNode;
  title?: string;
  description?: string;
}

function ProtectedDocsLayout({ children, title, description }: ProtectedDocsLayoutProps) {
  return (
    <Layout title={title} description={description}>
      <ProtectedRoute>
        {children}
      </ProtectedRoute>
    </Layout>
  );
}

export default ProtectedDocsLayout;
