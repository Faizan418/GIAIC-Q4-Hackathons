// Text selection utility for detecting and handling text selections

export interface TextSelectionResult {
  content: string;
  context: string;
  position: { start: number; end: number };
  sourceUrl: string;
}

export const getTextSelection = (): TextSelectionResult | null => {
  const selection = window.getSelection();
  if (!selection || selection.toString().trim() === '') {
    return null;
  }

  const selectedText = selection.toString().trim();
  if (selectedText.length === 0) {
    return null;
  }

  // Get the selected range
  const range = selection.getRangeAt(0);
  if (!range) {
    return null;
  }

  // Get surrounding context (before and after selection)
  const surroundingText = getSurroundingContext(range);

  return {
    content: selectedText,
    context: surroundingText,
    position: {
      start: range.startOffset,
      end: range.endOffset,
    },
    sourceUrl: window.location.href,
  };
};

// Get surrounding context of the selected text
const getSurroundingContext = (range: Range): string => {
  try {
    // Create ranges for text before and after the selection
    const startRange = document.createRange();
    const endRange = document.createRange();

    const container = range.commonAncestorContainer;

    // Set start range from beginning of container to start of selection
    startRange.setStart(container, 0);
    startRange.setEnd(range.startContainer, range.startOffset);

    // Set end range from end of selection to end of container
    endRange.setStart(range.endContainer, range.endOffset);
    endRange.setEnd(container,
      container.nodeType === Node.TEXT_NODE ?
        (container.textContent?.length || 0) :
        (container.childNodes.length || 0));

    // Get the text content around the selection (limit to reasonable length)
    const beforeText = startRange.toString().substring(0, 200);
    const afterText = endRange.toString().substring(0, 200);

    return `${beforeText}${range.toString()}${afterText}`.trim();
  } catch (error) {
    // Fallback if we can't get precise context
    return range.toString().trim();
  }
};

// Add event listener for text selection
export const addTextSelectionListener = (callback: (selection: TextSelectionResult | null) => void): (() => void) => {
  let selectionTimeout: number | null = null;

  const handler = () => {
    if (selectionTimeout) {
      window.clearTimeout(selectionTimeout);
    }

    // Use a timeout to debounce the selection event
    selectionTimeout = window.setTimeout(() => {
      const selection = getTextSelection();
      callback(selection);
    }, 150);
  };

  document.addEventListener('mouseup', handler);
  document.addEventListener('keyup', handler);

  // Return cleanup function
  return () => {
    document.removeEventListener('mouseup', handler);
    document.removeEventListener('keyup', handler);
    if (selectionTimeout) {
      window.clearTimeout(selectionTimeout);
    }
  };
};

// Get selected text with visual feedback
export const getSelectedTextWithFeedback = (): TextSelectionResult | null => {
  const selection = getTextSelection();

  if (selection) {
    // Add visual feedback by highlighting the selection
    const range = window.getSelection()?.getRangeAt(0);
    if (range) {
      // Create a temporary element to highlight the selection
      const highlight = document.createElement('span');
      highlight.style.backgroundColor = 'rgba(130, 180, 255, 0.3)';
      highlight.style.borderRadius = '2px';
      highlight.style.padding = '0 1px';

      try {
        range.surroundContents(highlight);

        // Remove highlight after a short delay
        setTimeout(() => {
          if (highlight.parentNode) {
            highlight.parentNode.replaceChild(highlight.firstChild!, highlight);
          }
        }, 1000);
      } catch (e) {
        // If surrounding fails (e.g., partially selected nodes), continue without visual feedback
        console.debug('Could not add visual feedback for selection:', e);
      }
    }
  }

  return selection;
};
