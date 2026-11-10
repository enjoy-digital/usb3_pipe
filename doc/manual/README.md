# USB3-PIPE manual sources

This directory holds the sources of the USB3-PIPE manual, the single reference document of the
project: USB 3 primer, PHY and device core architecture, supported FPGAs and boards, integration,
examples, simulation and debugging. It is written in Markdown and built with pandoc into a PDF
(eisvogel template) and a self-contained HTML page, both styled after enjoy-digital.fr. The figures
are SVG generated from Python sources in `doc/diagrams/`.

## Building

Requirements: `pandoc` (2.9 or 3.x), a TeX distribution with `xelatex` (Ubuntu:
`texlive-xetex texlive-latex-extra texlive-fonts-recommended texlive-fonts-extra lmodern`), and
`librsvg2-bin` (`rsvg-convert`) or the `cairosvg` Python package for the SVG figures. Fonts:
`fonts-crosextra-carlito` for the figures. Without `texlive-fonts-extra` (Source Sans Pro / Source
Code Pro), use `--light-fonts`. The Gravur title font of enjoy-digital.fr is not in the repository:
`build.py` downloads it into `build/doc/fonts/` (`--font-url` or `MANUAL_FONT_URL` to change the
source) and falls back to TeX Gyre Heros Condensed (PDF) and Arial Narrow (HTML) when it cannot.

```sh
python3 doc/manual/build.py                       # figures, checks, PDF, HTML
python3 doc/manual/build.py --pdf                 # PDF only
python3 doc/manual/build.py --html --check        # HTML plus the link/figure/table check
python3 doc/diagrams/build.py                     # regenerate the SVG figures (and PNG previews)
python3 doc/diagrams/build.py --check             # fail if a committed SVG is stale
```

Outputs are written to `build/doc/usb3_pipe_manual.pdf` and `build/doc/usb3_pipe_manual.html`;
they are never committed.

## Continuous integration and nightly manual

`.github/workflows/doc.yml` runs two jobs:

- `check`, on every push and pull request: `doc/diagrams/build.py --check` (figures up to date, no
  layout warning) and `doc/manual/build.py --check` (links, anchors, images, table widths).
- `nightly`, every night on `master` (and on demand with `gh workflow run doc.yml`): builds the PDF
  and HTML and publishes them on the rolling
  [`manual-nightly`](https://github.com/enjoy-digital/usb3_pipe/releases/tag/manual-nightly)
  pre-release, whose notes record the commit. A night without a new commit on `master` is skipped.

## Layout

| Path | Content |
|---|---|
| `NN_*.md`, `A_*.md` to `E_*.md` | One file per chapter or appendix, in the order of `defaults-pdf.yaml` |
| `metadata.yaml` | Title page, theme colours and fonts of the PDF, shared by PDF and HTML |
| `defaults-pdf.yaml`, `defaults-html.yaml` | pandoc defaults files (input list, template, output); `build.py` rewrites them with absolute paths |
| `template/eisvogel.latex` | Vendored eisvogel template and its BSD license (title font hook added) |
| `template/manual.html5` | HTML page template (title band, sticky table of contents) |
| `style.css` | Style of the HTML page |
| `figures/` | Raster assets (logos; `litex_logo_dark.png` is `doc/images/litex-logo.png` with a light tagline for dark backgrounds) |
| `tools/` | Table converter, cover generator, register appendix generator (`csr_doc.py`), pandoc Lua filters |
| `build.py`, `check.py` | Build driver and source checker |
| `../diagrams/` | Python sources of the SVG figures, `svgdiag.py` helper, `svg/` outputs |
| `SOURCES.md` | Where each chapter's facts come from (files to re-check when the code changes) |
| `STYLE.md` | Editorial style guide |

## Writing rules

`STYLE.md` is the editorial style guide; the rules below are the mechanical ones.

- Every fact (class, parameter, width, rate, default) must be traceable to a file of the
  repository at the documented revision. Prefer quoting the source path in the text when a reader
  may need to check it.
- Give stable identifiers to headings that are cross-referenced (`## Clock domains {#clock-domains}`)
  and link with `[text](#clock-domains)`; `check.py` fails on unresolved anchors.
- Figures: reference the SVG directly (`![Caption](phy_architecture.svg)`), no PNG. Photos of
  `doc/` are referenced by file name.
- Code and commands go in fenced blocks with a language (`sh`, `console`, `python`, `verilog`).
- Do not hand-edit SVGs: `svgdiag.py` encodes the figure style.

## Theme

The theme follows enjoy-digital.fr and its blog: brand cyan `#2eaadc` (numbers, rules, table
heads, code block rule, figure accents), text blue `#136f9c` (links), near-black `#0a0a0a`
headings, the dark `#010101` title band with its faint cyan grid, dark code blocks (`#0d1117`),
and the Gravur display font for titles. The values live in `metadata.yaml` (PDF), `style.css`
(HTML), `tools/cover.py` (PDF title page) and `../diagrams/svgdiag.py` (figures).

## pandoc notes

The sources build with pandoc 2.9 and 3.x. `build.py` converts every pipe table into a grid table
(`tools/tables.py`) so that both versions give the same column widths, converts the SVG figures
to PDF for LaTeX (`tools/svg2pdf.lua`, no `rsvg-convert` needed at pandoc time) and lets long
inline code break (`tools/breakcode.lua`). `metadata.yaml` sets `book: true` and
`classoption: [oneside, openany]`, and bounds the figures to the text block with `\maxwidth` and
`\maxheight` (pandoc 2.9 emits a bare `\includegraphics`).
