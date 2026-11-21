"""Minimal InfluxDB 1.x query helper. Credentials from $INFLUX_HA_VAN_USER / $INFLUX_HA_VAN_PASS."""
import json
import os
import urllib.parse
import urllib.request

URL = os.environ.get('INFLUX_URL', 'http://tm.fabi.me:8086/query')


def query(q, epoch='ms', db='ha_van'):
    params = urllib.parse.urlencode({'db': db, 'q': q, 'epoch': epoch,
                                     'u': os.environ['INFLUX_HA_VAN_USER'],
                                     'p': os.environ['INFLUX_HA_VAN_PASS']})
    with urllib.request.urlopen(URL + '?' + params, timeout=300) as r:
        d = json.load(r)
    out = []
    for res in d['results']:
        if 'error' in res:
            raise RuntimeError(res['error'])
        for s in res.get('series', []):
            out.append(s)
    return out


def query_multi(qs, epoch='ms', db='ha_van'):
    """several statements in one request; returns one list of series per statement"""
    params = urllib.parse.urlencode({'db': db, 'q': ';'.join(qs), 'epoch': epoch,
                                     'u': os.environ['INFLUX_HA_VAN_USER'],
                                     'p': os.environ['INFLUX_HA_VAN_PASS']})
    with urllib.request.urlopen(URL + '?' + params, timeout=300) as r:
        d = json.load(r)
    out = [[] for _ in qs]
    for res in d['results']:
        if 'error' in res:
            raise RuntimeError(res['error'])
        out[res['statement_id']] = res.get('series', [])
    return out
