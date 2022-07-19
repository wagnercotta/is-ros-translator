import sys
import json
from core.rostranslator_publisher import ROSTranslator_Publisher


def main():
    config_file = sys.argv[1] if len(sys.argv) > 1 else 'etc/conf/config.json'
    config = json.load(open(config_file, 'r'))

    broker_uri = config['broker_uri']
    translator_id = config['translator_id']

    service = ROSTranslator_Publisher(translator_id,broker_uri)
    service.run()

if __name__ == "__main__":
    main()