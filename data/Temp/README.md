# Active-run staging

Steering configs write error, command, observer weights/traces, and transient
logs here while a process is active. The recorded-run wrapper moves the files
into the final timestamped `data/steering_runs/` folder when the process exits.

Observer weights are emitted by `fsp_fini`, so they appear on normal completion
or clean SIGINT/Ctrl-C—not exactly when control transitions from open to closed.
SIGKILL and crashes can leave weights missing or empty. Configs with
`broad_order: 0` have no learned broadband weights; an empty weight file is
therefore expected.
