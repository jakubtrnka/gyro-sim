import {
  cpSync,
  existsSync,
  mkdirSync,
  readFileSync,
  writeFileSync,
} from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";
import MarkdownIt from "markdown-it";
import { katex } from "@mdit/plugin-katex";
import GithubSlugger from "github-slugger";
import { chromium } from "@playwright/test";

const root = fileURLToPath(new URL("../../", import.meta.url));
const out = path.join(root, "web/dist");
const documents = [
  { source: "docs/01-gyroscope-mechanics.md", label: "Gyroscope mechanics" },
  { source: "docs/02-simulation-design.md", label: "Simulation design" },
  { source: "README.md", label: "Project guide" },
  { source: "gyroscope-simulation-design.md", label: "Original proposal" },
];
const sources = new Set(documents.map(({ source }) => source));
const htmlPath = (source) => source.replace(/\.md$/, ".html");
const relative = (source, target) =>
  path.posix.relative(path.posix.dirname(source), target);

// Keep the source downloads and linked code examples alongside the HTML pages.
for (const source of [
  "README.md",
  "gyroscope-simulation-design.md",
  "docs",
  "crates/gyro-core/tests",
  "crates/gyro-core/examples",
  "crates/gyro-wasm/src",
  "crates/gyro-wasm/examples",
  "web/tests",
]) {
  const dest = path.join(out, source);
  mkdirSync(path.dirname(dest), { recursive: true });
  cpSync(path.join(root, source), dest, { recursive: true });
}
const assets = path.join(out, "docs/assets");
mkdirSync(assets, { recursive: true });
cpSync(new URL("./docs.css", import.meta.url), path.join(assets, "docs.css"));
const katexDist = fileURLToPath(new URL("./", import.meta.resolve("katex")));
cpSync(
  path.join(katexDist, "katex.min.css"),
  path.join(assets, "katex.min.css"),
);
cpSync(path.join(katexDist, "fonts"), path.join(assets, "fonts"), {
  recursive: true,
});
cpSync(
  path.join(katexDist, "../LICENSE"),
  path.join(assets, "KaTeX-LICENSE.txt"),
);

const md = new MarkdownIt({ html: false }).use(katex, {
  throwOnError: true,
  logger: (code) => (code === "newLineInDisplayMode" ? "ignore" : "error"),
  trust: false,
});
const escape = md.utils.escapeHtml;
md.renderer.rules.table_open = () =>
  '<div class="table-scroll" tabindex="0"><table>\n';
md.renderer.rules.table_close = () => "</table></div>\n";

function rewriteLinks(tokens, source) {
  for (const token of tokens) {
    if (token.type === "link_open") {
      const href = token.attrGet("href");
      // Only rewrite links to documents that this build actually renders.
      if (href && !/^(?:[a-z][a-z\d+.-]*:|\/|#)/i.test(href)) {
        const target = href.split(/[?#]/, 1)[0];
        const resolved = path.posix.normalize(
          path.posix.join(
            path.posix.dirname(source),
            decodeURIComponent(target),
          ),
        );
        if (sources.has(resolved))
          token.attrSet("href", href.replace(/\.md(?=[?#]|$)/, ".html"));
      }
    }
    if (token.children) rewriteLinks(token.children, source);
  }
}

function layout(source, title, content, headings) {
  const link = (target) => escape(relative(source, target));
  const home = link("index.html");
  const navigation = documents
    .map(
      (document) =>
        `<a href="${link(htmlPath(document.source))}"${document.source === source ? ' aria-current="page"' : ""}>${escape(document.label)}</a>`,
    )
    .join("\n");
  const contents = headings
    .map(
      ({ id, text }) => `<li><a href="#${escape(id)}">${escape(text)}</a></li>`,
    )
    .join("\n");
  return `<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="color-scheme" content="light">
  <title>${escape(title)} · Gyroscope Lab</title>
  <link rel="stylesheet" href="${link("docs/assets/katex.min.css")}">
  <link rel="stylesheet" href="${link("docs/assets/docs.css")}">
</head>
<body>
  <a class="skip-link" href="#document">Skip to content</a>
  <header class="masthead">
    <a class="brand" href="${home}">GYROSCOPE <span>/ LAB</span></a>
    <a class="back-link" href="${home}">← Back to simulation</a>
  </header>
  <div class="page-layout">
    <aside>
      <nav aria-label="Documentation" class="documents">
        <p class="eyebrow">The reading room</p>
        ${navigation}
      </nav>
      <nav aria-label="On this page" class="contents">
        <p class="eyebrow">On this page</p>
        <ol>${contents}</ol>
      </nav>
      <details class="mobile-contents">
        <summary>On this page</summary>
        <nav aria-label="On this page"><ol>${contents}</ol></nav>
      </details>
    </aside>
    <main id="document" tabindex="-1">
      <article>${content}</article>
      <footer><a href="${home}">Explore the interactive simulation →</a></footer>
    </main>
  </div>
</body>
</html>
`;
}

// Render diagrams once in the build. The published documentation needs no JS,
// browser engine, CDN, or network requests beyond its own CSS and math fonts.
const browser = await chromium.launch({
  executablePath:
    process.env.CHROMIUM_PATH ??
    (existsSync("/usr/bin/chromium") ? "/usr/bin/chromium" : undefined),
});
try {
  const page = await browser.newPage({
    viewport: { width: 1200, height: 900 },
  });
  await page.route("**/*", (route) => route.abort());
  await page.setContent('<!doctype html><html lang="en"><body></body></html>');
  const mermaidScript = fileURLToPath(
    new URL("mermaid.min.js", import.meta.resolve("mermaid")),
  );
  await page.addScriptTag({ path: mermaidScript });
  await page.evaluate(() => {
    window.mermaid.initialize({
      startOnLoad: false,
      securityLevel: "strict",
      deterministicIds: true,
      deterministicIDSeed: "gyro-docs",
      theme: "base",
      fontFamily: "Arial, sans-serif",
      themeVariables: {
        primaryColor: "#edf3ee",
        primaryTextColor: "#1d3441",
        primaryBorderColor: "#278e8a",
        lineColor: "#687b80",
        fontFamily: "Arial, sans-serif",
      },
      flowchart: { htmlLabels: false },
    });
  });
  for (const { source } of documents) {
    const tokens = md.parse(readFileSync(path.join(root, source), "utf8"), {});
    rewriteLinks(tokens, source);
    const slugger = new GithubSlugger();
    const headings = [];
    let title;
    let section = "";
    let diagram = 0;
    for (let i = 0; i < tokens.length; i++) {
      const token = tokens[i];
      if (token.type === "heading_open") {
        const text = tokens[i + 1].children
          .filter(
            (child) =>
              !child.type.endsWith("_open") && !child.type.endsWith("_close"),
          )
          .map((child) => child.content)
          .join("");
        const id = slugger.slug(text);
        token.attrSet("id", id);
        if (token.tag === "h1") title ??= text;
        if (token.tag === "h2") headings.push({ id, text });
        section = text;
      }
      if (token.type === "fence" && token.info.trim() === "mermaid") {
        const svg = await page.evaluate(
          async ({ id, definition }) =>
            (await window.mermaid.render(id, definition)).svg,
          { id: `diagram-${diagram++}`, definition: token.content },
        );
        token.type = "html_block";
        token.content = `<figure class="diagram" tabindex="0" aria-label="${escape(section)}">${svg}</figure>\n`;
      }
    }
    if (!title) throw new Error(`Missing document title: ${source}`);
    const content = md.renderer.render(tokens, md.options, {});
    // Math plugins may catch parse errors and emit an error span: fail the build.
    if (/class=["'][^"']*\bkatex-error\b/.test(content))
      throw new Error(`Invalid equation in ${source}`);
    writeFileSync(
      path.join(out, htmlPath(source)),
      layout(source, title, content, headings),
    );
    console.log(
      `Rendered ${source}: ${headings.length} sections, ${diagram} diagrams`,
    );
  }
} finally {
  await browser.close();
}
