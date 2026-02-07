import type { Metadata } from "next";
import { Poppins } from "next/font/google";
import { ThemeProvider } from "next-themes";
import { Analytics } from "@vercel/analytics/react";
import { SpeedInsights } from "@vercel/speed-insights/next";
import { ReactQueryProvider } from "@/providers/QueryProvider";
import "@/styles/globals.css";
import Navbar from "@/components/navigation/Navbar";

const poppins = Poppins({
  subsets: ["latin"],
  weight: ['400', '500', '600', '700'],
});

export const metadata: Metadata = {
  title: {
    default: "TASKAPP - Master Your Day",
    template: "%s | TASKAPP"
  },
  description: "Professional Todo Application with beautiful UI and powerful features",
  keywords: ["task management", "todo app", "productivity", "task tracker", "organizer"],
  authors: [{ name: "TASKAPP Team" }],
  creator: "TASKAPP Team",
  publisher: "TASKAPP Team",
  formatDetection: {
    email: false,
    address: false,
    telephone: false,
  },
  openGraph: {
    type: "website",
    locale: "en_US",
    url: "https://www.taskapp.example.com",
    title: "TASKAPP - Master Your Day",
    description: "Professional Todo Application with beautiful UI and powerful features",
    siteName: "TASKAPP",
  },
  twitter: {
    card: "summary_large_image",
    title: "TASKAPP - Master Your Day",
    description: "Professional Todo Application with beautiful UI and powerful features",
  },
  robots: {
    index: true,
    follow: true,
    googleBot: {
      index: true,
      follow: true,
      'max-video-preview': -1,
      'max-image-preview': 'large',
      'max-snippet': -1,
    },
  },
  icons: {
    icon: "/favicon.ico",
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body className={`${poppins.className} antialiased`}>
        <ReactQueryProvider>
          <ThemeProvider
            attribute="class"
            defaultTheme="system"
            enableSystem
            disableTransitionOnChange
          >
            <Navbar />
            <main className="pt-16 min-h-screen">{children}</main>
            <Analytics />
            <SpeedInsights />
          </ThemeProvider>
        </ReactQueryProvider>
      </body>
    </html>
  );
}