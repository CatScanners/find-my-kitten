---
parent: Development
---

# Creating documentation with GitHub pages

The markdown files under `/docs/` of the find-my-kitten repository use GitHub pages with a remote theme defined in `_config.yml`.
```yml
# contents of _config.yml
title: AI, find my kitten! | Documentation
remote_theme: just-the-docs/just-the-docs
color_scheme: dark
```

The result is static pages running on [Jekyll](<https://jekyllrb.com/>) with the just-the-docs theme. Documentation for the theme can be found on <https://just-the-docs.com/> but this page serves as a quick rundown.

Jekyll will turn any .md file into an HTML page with MD formatting. An implicit header is defined from file contents, but it is possible to write some parameters explicitly if desired. An example header (at the start of the md file) could look like the following:
```
---
title:
---
```

Some notes:
- A link must be specified with angle brackets, e.g. `<https://github.com/CatScanners/find-my-kitten>`