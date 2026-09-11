"""Direct transport settings and exact boot confirmation; no live converter."""
import asyncio
import hashlib
from types import SimpleNamespace as NS
import unittest
from unittest.mock import AsyncMock, patch
import ota_ble as P


class Integration(unittest.IsolatedAsyncioTestCase):
    def args(self, **kw):
        values = dict(name='fugu-bench', address=None, ble_proxy=None, proxy_password=None)
        values.update(kw)
        return NS(**values)

    async def test_direct_settings_reach_shared_link(self):
        link = P.make_link(self.args(ble_backend='bumble', adapter='hci0', chunk=495,
                                     ble_interval_ms=12.5, ble_phy=2))
        self.assertEqual(link._link.options, P.T.Options('bumble', 'hci0', 495, 12.5, 2))
        link._link.prepare_transfer = AsyncMock()
        await P.O.adapt_link(link).prepare_transfer()
        link._link.prepare_transfer.assert_awaited_once()
        self.assertEqual((link._link.cmd_uuid, link._link.notify_uuid, link._link.fw_uuid),
                         (P.RX_UUID, P.TX_UUID, P.FW_UUID))

    async def test_proxy_defaults_survive_but_direct_tuning_is_rejected(self):
        self.assertIsInstance(P.make_link(self.args(ble_proxy='proxy.local')), P.ProxyLink)
        with self.assertRaises(ValueError):
            P.make_link(self.args(ble_proxy='proxy.local', ble_backend='native'))

    async def test_boot_verification_rejects_wrong_missing_and_same_slot(self):
        data = b'x' * 256
        data += hashlib.sha256(data).digest()
        before = dict(run='app0')
        target = P.O.image_id(data)
        for info, expected in [(None, False), (dict(base=target), False),
                               (dict(base=target, run='app0'), False),
                               (dict(base='f'*64, run='app1'), False),
                               (dict(base=target, run='app1'), True)]:
            link = P.BleakLink('fugu-bench')
            link.open = AsyncMock()
            link.release = AsyncMock()
            with self.subTest(info=info), patch.object(P.O, 'query_info', AsyncMock(return_value=info)):
                result = await link.verify(data, before, total_timeout=.005)
            self.assertEqual(result, expected)
            link.release.assert_awaited()


if __name__ == '__main__': unittest.main()
