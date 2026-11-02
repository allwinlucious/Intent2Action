# Phase V7 design notes

The demo is now a quiet two-column tool: request, reading, and program form one flow; the robot view, call log, and result form the other. The header holds only scene controls, faults, waiting state, and Stop.

Removed the progress display, cards, chips, decorative labels, overlays, placeholder panels, gradients, shadows, and non-semantic colour. Readings and programs appear only when available. Questions and patch approval share one amber waiting block above the program.

All existing requests, run gates, stream handling, clip recording, revisions, and approval controls remain in place. Forms now have explicit labels, names and autocomplete behavior; async errors are announced and focusable; keyboard focus is visible; light mode follows the system preference.

The filming layout now gives the robot view the full window, with a quiet right-hand sidebar for the complete request-to-result flow and one persistent play control over the view. The sidebar opens on first load, supports its toggle and keyboard shortcuts, and opens itself at the amber waiting block whenever a person must respond.
