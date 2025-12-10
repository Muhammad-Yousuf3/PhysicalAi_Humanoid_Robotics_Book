# Quickstart: Physical AI Book Development

## Prerequisites
-   Node.js version 18.0 or above
-   Git

## Installation

1.  **Clone the repository**:
    ```bash
    git clone <repo-url>
    cd PhysicalAi_Humanoid_Robotics_Book
    ```

2.  **Install dependencies**:
    ```bash
    npm install
    ```

## Local Development

Start the development server:
```bash
npm start
```
-   Opens http://localhost:3000
-   Hot reloading enabled.

## Building

Generate static files for production:
```bash
npm run build
```
Output directory: `build/`

## Content Workflow

1.  **Create Spec**: `bin/create-spec.sh <feature-name>`
2.  **Write Content**: Create `.md` files in `docs/` referencing the spec.
3.  **Validate**: `npm run verify-specs` (Custom script)
4.  **Commit**: `git commit -m "docs: add chapter X"`

## Theme Customization

Edit `src/css/custom.css` to tweak the Neon Cyber AI theme variables.
