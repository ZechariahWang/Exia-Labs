#!/bin/bash
set -e

KEY_DIR="$HOME/.exia"
PRIVATE_KEY="$KEY_DIR/radio_private.pem"
PUBLIC_KEY="$KEY_DIR/radio_public.pem"

mkdir -p "$KEY_DIR"

if [ -f "$PRIVATE_KEY" ] || [ -f "$PUBLIC_KEY" ]; then
    echo "Keys already exist in $KEY_DIR"
    echo "  $PRIVATE_KEY"
    echo "  $PUBLIC_KEY"
    read -p "Overwrite? [y/N] " confirm
    if [ "$confirm" != "y" ] && [ "$confirm" != "Y" ]; then
        echo "Aborted."
        exit 0
    fi
fi

openssl genpkey -algorithm RSA -pkeyopt rsa_keygen_bits:2048 -out "$PRIVATE_KEY" 2>/dev/null
openssl rsa -pubout -in "$PRIVATE_KEY" -out "$PUBLIC_KEY" 2>/dev/null
chmod 600 "$PRIVATE_KEY"
chmod 644 "$PUBLIC_KEY"

echo "RSA-2048 key pair generated:"
echo "  Private: $PRIVATE_KEY"
echo "  Public:  $PUBLIC_KEY"
echo ""
echo "Copy both files to ~/.exia/ on the other machine."
