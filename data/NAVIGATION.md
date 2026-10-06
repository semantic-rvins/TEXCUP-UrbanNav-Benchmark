# Broadcast-navigation inputs

`brdm1290.19p` and `brdm1290_v304.19p` are independent mixed-GNSS broadcast
navigation inputs for day 129 of 2019. They are separate from the TEX-CUP
receiver recordings and are retained through Git LFS. The RTKLIB observation
conversion recipe does not generate or replace either navigation product.

Mixed broadcast-navigation products are available through the
[BKG IGS broadcast archive](https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2019/129/).
The original download URL for the historical files was not recorded, so this
link identifies the public product archive rather than asserting a freshly
verified, byte-identical upstream download.

The second file is the existing RINEX 3.04-compatible adaptation for the GVINS
navigation parser. It is a modified input; do not represent it as the original
broadcast product. Historical run provenance identifies which variant each
method consumed. This change does not regenerate navigation or alter those
saved results.

From the repository root, using GitHub credentials that permit LFS access:

```bash
git lfs install
git lfs pull --include="data/tex_cup/*.19p"
```

A Git LFS pointer is not a RINEX navigation file; complete retrieval before
running an estimator.
