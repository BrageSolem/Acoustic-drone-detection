FROM node:18-alpine

WORKDIR /app

# Install runtime deps needed for HEALTHCHECK (curl)
RUN apk add --no-cache curl

# Copy package manifests first for better caching
COPY package*.json ./

# Install only production dependencies
ENV NODE_ENV=production
RUN npm install --production

# Copy app source
COPY . .

# Use non-root user (node exists in official image)
USER node

EXPOSE 3000

HEALTHCHECK --interval=30s --timeout=5s --start-period=5s --retries=3 CMD curl -f http://localhost:3000/health || exit 1

CMD ["node", "server.js"]
