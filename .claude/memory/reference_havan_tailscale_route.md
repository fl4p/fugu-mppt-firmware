---
name: reference_havan_tailscale_route
description: Reach havan + fry/flat telnet from off-site via Tailscale 100.64.0.10 (havan.local doesn't resolve off-LAN)
metadata:
  type: reference
---

Off the van LAN, `havan.local` doesn't resolve but havan is on Tailscale at **100.64.0.10**. Use `ssh -o HostKeyAlias=havan.local 100.64.0.10` (verifies against the known havan.local key; bare IP fails host-key check). From havan the NAT router 192.168.1.231 and broker 192.168.1.200 are reachable, so tunnel telnet: `ssh -f -N -o HostKeyAlias=havan.local -L 12232:192.168.1.231:232 ... 100.64.0.10`, then `etc/fugu_console.py --ip 127.0.0.1:12232 --stdin`. Ports flip-flop ([[project_fry_flat_nat_mapping_swapped]]) — confirm `hostname`. Console log: `pv/fugu_console.log` on havan.
