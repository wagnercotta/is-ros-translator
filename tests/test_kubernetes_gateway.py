"""Regression checks for a stand-alone Kubernetes PrIS gateway."""

from pathlib import Path
import sys
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
MANIFESTS = ROOT / 'deploy' / 'kubernetes'


def manifest(name):
    return yaml.safe_load((MANIFESTS / name).read_text())


class KubernetesGatewayTests(unittest.TestCase):
    def test_kustomization_has_only_gateway_resources(self):
        files = manifest('kustomization.yaml')['resources']
        self.assertEqual(set(files), {'namespace.yaml', 'configmap.yaml',
                                      'secret-local.yaml', 'deployment.yaml', 'service.yaml'})
        self.assertTrue(all((MANIFESTS / name).exists() for name in files))

    def test_robots_are_independently_discoverable(self):
        from is_ros_translator_v2.config import Settings

        cfg = yaml.safe_load(manifest('configmap.yaml')['data']['gateway.yaml'])
        settings = Settings.model_validate(cfg)
        self.assertEqual(len(settings.robots), 4)
        self.assertEqual(sorted(r.ros_version for r in settings.robots.values()), [1, 1, 2, 2])
        self.assertEqual(len({r.host for r in settings.robots.values()}), 4)
        self.assertTrue(all(r.port == 9090 for r in settings.robots.values()))
        self.assertTrue(all(r.host.startswith('rosbridge.') for r in settings.robots.values()))
        self.assertTrue(settings.iswire.enabled)
        for rid, cfg in settings.robots.items():
            self.assertEqual(cfg.navigation.action,
                             '/move_base' if cfg.ros_version == 1 else '/navigate_to_pose')

    def test_gateway_has_one_replica_and_no_ros_sidecar(self):
        deploy = manifest('deployment.yaml')
        self.assertEqual(deploy['spec']['replicas'], 1)
        self.assertEqual(deploy['spec']['strategy']['type'], 'Recreate')
        containers = deploy['spec']['template']['spec']['containers']
        self.assertEqual([c['name'] for c in containers], ['gateway'])
        self.assertEqual(containers[0]['command'], ['is-ros-translator'])
        self.assertEqual(containers[0]['args'],
                         ['--config', '/etc/is-ros-translator/gateway.yaml'])
        self.assertEqual(containers[0]['livenessProbe']['httpGet']['path'], '/health/live')
        self.assertEqual(containers[0]['startupProbe']['httpGet']['path'], '/health/live')
        self.assertIn('"status"', containers[0]['readinessProbe']['exec']['command'][-1])
        self.assertIn('ISWIRE_BROKER_URI', [e['name'] for e in containers[0]['env']])

    def test_service_is_private_and_correctly_routes_http(self):
        service = manifest('service.yaml')
        self.assertEqual(service['spec']['type'], 'ClusterIP')
        self.assertEqual(service['spec']['ports'][0]['port'], 8080)
        self.assertEqual(service['spec']['selector'],
                         manifest('deployment.yaml')['spec']['template']['metadata']['labels'])

    def test_no_external_dependencies_deployed(self):
        resources = [manifest(name) for name in manifest('kustomization.yaml')['resources']]
        kinds = {resource['kind'] for resource in resources}
        self.assertEqual(kinds, {'Namespace', 'ConfigMap', 'Secret', 'Deployment', 'Service'})
        self.assertEqual(manifest('secret-local.yaml')['stringData']['GATEWAY_API_KEY'],
                         'local-e2e-api-key')


if __name__ == '__main__':
    unittest.main()
