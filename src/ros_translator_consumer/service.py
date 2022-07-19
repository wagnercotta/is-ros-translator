import sys
import json
from core.rostranslator_consumer import ROSTranslator_Consumer


def main():

    config_file = sys.argv[1] if len(sys.argv) > 1 else 'etc/conf/config.json'
    config = json.load(open(config_file, 'r'))

    broker_uri = config['broker_uri']
    management_url = config['management_url']
    translator_id = config['translator_id']

    service = ROSTranslator_Consumer(translator_id,broker_uri,management_url)
    service.run()

if __name__ == "__main__":
    main()