# `capra_near_18_v1` trial 10 — 5-rep validation (lucky in-optim, do not deploy)

## Why this run exists

After trial 6 (near_18, in-optim q75=0.0702) was 5-rep validated and
flagged as new deployment winner, trial 10 (in-optim q75=0.0715) was
the second-best. Quick validation to see if it's an alternative
operating point or just a lucky single-trial sample.

## 5-rep result

```
median worst-bag: 0.292 (trial 22: 0.177, near_18 t6: 0.128)
median q75:       0.124 (trial 22: 0.087, near_18 t6: 0.0713)
max worst-bag:    0.654
bag failures:     0 (no hard failures, but moving_long_bag3 drift hits 0.55-0.65 in 3 of 5 reps)
```

Trial 10 catastrophically loses tracking on `moving_long_bag3` in
multiple reps (drift 0.56, 0.29, 0.65 — vs trial 6's tight 0.017).
**Trial 10 is a lucky in-optim sample.** Even with n_reps=5 in-optim,
its 5 in-optim reps happened to miss the bag3 failure mode.

## Verdict

**Do not deploy.** Trial 6 (near_18) remains the clean winner.

This is another data point: **n_reps=5 in-optim is more honest than
n_reps=3 but still does not guarantee robustness across an independent
5-rep validation**. The 10-validation-rep (n_reps=10) pattern would be
needed to fully characterize, but at 2× wall cost.

The lesson for any future deployment-candidate decision: **always 5-rep
validate independently of optim, even when in-optim used n_reps ≥ 5**.

Trial 6's much smaller in-optim-to-validation gap (1.6%) means its
basin is genuinely robust; trial 10's larger gap (in-optim 0.072 → 5-rep
0.124, ~70%) means its in-optim was lucky.
