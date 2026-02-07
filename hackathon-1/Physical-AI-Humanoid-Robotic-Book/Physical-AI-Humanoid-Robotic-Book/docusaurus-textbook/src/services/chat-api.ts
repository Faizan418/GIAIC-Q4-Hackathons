// API service for chat functionality
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import {
  QueryRequest,
  QueryResponse,
  TextSelectionQueryRequest,
  TextSelectionQueryResponse
} from '../types/chat';

export function useChatApi() {
  const { siteConfig } = useDocusaurusContext();

  const API_BASE_URL = siteConfig.customFields
    ?.CHAT_API_URL as string;

  if (!API_BASE_URL) {
    throw new Error("CHAT_API_URL is not defined");
  }

  return {
    async query(request: QueryRequest): Promise<QueryResponse> {
      try {
        const response = await fetch(`${API_BASE_URL}/api/query`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(request),
        });

        if (!response.ok) {
          throw new Error(`API request failed with status ${response.status}`);
        }

        const data = await response.json();
        return data;
      } catch (error) {
        console.error('Error in query API call:', error);
        throw error;
      }
    },

    async textSelectionQuery(request: TextSelectionQueryRequest): Promise<TextSelectionQueryResponse> {
      try {
        const response = await fetch(`${API_BASE_URL}/api/text-selection-query`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(request),
        });

        if (!response.ok) {
          throw new Error(`API request failed with status ${response.status}`);
        }

        const data = await response.json();
        return data;
      } catch (error) {
        console.error('Error in text selection query API call:', error);
        throw error;
      }
    }
  };
}
