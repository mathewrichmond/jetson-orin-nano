#!/bin/bash
# Container deployment script for Isaac robot microservices

set -e

CMD="${1:-status}"
SERVICE="${2:-}"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

check_docker() {
    if ! docker compose version &> /dev/null; then
        echo -e "${RED}Error: docker compose not available${NC}"
        exit 1
    fi
}

case "$CMD" in
    build)
        echo -e "${GREEN}Building containers...${NC}"
        docker compose build $SERVICE
        ;;
    up|start)
        echo -e "${GREEN}Starting containers...${NC}"
        docker compose up -d $SERVICE
        ;;
    down|stop)
        echo -e "${YELLOW}Stopping containers...${NC}"
        docker compose down
        ;;
    restart)
        echo -e "${YELLOW}Restarting containers...${NC}"
        docker compose restart $SERVICE
        ;;
    logs)
        docker compose logs -f $SERVICE
        ;;
    ps|status)
        docker compose ps
        echo ""
        docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}" \
            $(docker compose ps -q) 2>/dev/null || true
        ;;
    *)
        echo "Usage: $0 {build|start|stop|restart|logs|status} [service]"
        echo ""
        echo "Services: robot-control, vla-inference, system-monitor"
        exit 1
        ;;
esac
