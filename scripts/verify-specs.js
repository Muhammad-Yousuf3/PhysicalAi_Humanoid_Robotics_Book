const fs = require('fs');
const path = require('path');

const DOCS_DIR = path.join(__dirname, '../docs');
const SPECS_DIR = path.join(__dirname, '../specs');

let hasError = false;

function getFiles(dir) {
  if (!fs.existsSync(dir)) return [];
  const subdirs = fs.readdirSync(dir);
  const files = subdirs.map((subdir) => {
    const res = path.resolve(dir, subdir);
    return (fs.statSync(res).isDirectory()) ? getFiles(res) : res;
  });
  return files.reduce((a, f) => a.concat(f), []);
}

console.log('Validating specs references...');

const files = getFiles(DOCS_DIR).filter(f => f.endsWith('.md') || f.endsWith('.mdx'));

files.forEach(file => {
  const content = fs.readFileSync(file, 'utf8');
  const frontmatterMatch = content.match(/^---\n([\s\S]*?)\n---/);
  
  if (!frontmatterMatch) {
    console.error(`ERROR: ${path.relative(process.cwd(), file)} missing frontmatter.`);
    hasError = true;
    return;
  }

  const frontmatter = frontmatterMatch[1];
  const specRefMatch = frontmatter.match(/^spec_ref:\s*(.+)$/m);

  if (!specRefMatch) {
    console.error(`ERROR: ${path.relative(process.cwd(), file)} missing 'spec_ref' in frontmatter.`);
    hasError = true;
    return;
  }

  const specRef = specRefMatch[1].trim().replace(/^['"]|['"]$/g, ''); // Remove quotes if present
  // Resolve specRef relative to repo root if it starts with specs/, or just resolve it
  const specPath = path.resolve(process.cwd(), specRef);

  if (!fs.existsSync(specPath)) {
    console.error(`ERROR: ${path.relative(process.cwd(), file)} references missing spec: ${specRef}`);
    hasError = true;
  }
});

if (hasError) {
  console.error('Validation failed.');
  process.exit(1);
} else {
  console.log('All specs validated successfully.');
}
